function result = tuneMY8077ExposureGain(webcamIdx, varargin)
% TUNEMY8077EXPOSUREGAIN  Quick exposure/gain tuner for the MY8077 webcam.
%
%   result = tuneMY8077ExposureGain()
%   result = tuneMY8077ExposureGain(webcamIdx)
%   result = tuneMY8077ExposureGain(..., 'Name', value)
%
%   Sweeps manual Exposure/Gain pairs for the MY8077, captures short bursts,
%   scores each pair, and prints a recommended setting. This is intentionally
%   standalone: run it before a session, inspect the saved candidate frames,
%   then copy the chosen Exposure/Gain into buildConfig if desired.
%
%   Example:
%     matlab -batch "addpath(genpath('Program')); tuneMY8077ExposureGain(2)"
%
%   Useful options:
%     'Roi'              [x y w h] metric crop. Default = full frame.
%     'ExposureValues'   default -7:-3.
%     'GainValues'       default 0:16:128.
%     'TargetMeanDN'     default 150.
%     'MaxBrightClipPct' default 1.0.
%     'MaxDarkClipPct'   default 1.0.
%     'FramesPerPair'    default 3.
%     'OutputDir'        output folder.
%
%   See also: buildConfig, applyCameraSettings

if nargin < 1 || isempty(webcamIdx)
    webcamIdx = 2;
end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'Roi', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 4));
addParameter(p, 'ExposureValues', -7:-3, @(x) isnumeric(x) && ~isempty(x));
addParameter(p, 'GainValues', 0:16:128, @(x) isnumeric(x) && ~isempty(x));
addParameter(p, 'TargetMeanDN', 150, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'MaxBrightClipPct', 1.0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'MaxDarkClipPct', 1.0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'FramesPerPair', 3, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(p, 'WarmupFrames', 3, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'Resolution', [1280 720], @(x) isnumeric(x) && numel(x) == 2);
addParameter(p, 'Brightness', 0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Contrast', 64, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Gamma', 300, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Saturation', 128, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Sharpness', 0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'OutputDir', '', @(x) ischar(x) || isa(x, 'string'));
parse(p, varargin{:});
opts = p.Results;
opts.OutputDir = toChar(opts.OutputDir);

rootDir = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(genpath(rootDir));
cfg = buildConfig();
if isempty(opts.OutputDir)
    opts.OutputDir = fullfile(rootDir, cfg.logDir, 'diagnostics', ...
        ['my8077_tune_' datestr(now, 'yyyymmdd_HHMMSS')]);
end
if ~isfolder(opts.OutputDir)
    mkdir(opts.OutputDir);
end

fprintf('MY8077 exposure/gain tuner\n');
fprintf('  webcam index: %d\n', webcamIdx);
fprintf('  resolution:   %dx%d\n', opts.Resolution(1), opts.Resolution(2));
fprintf('  output:       %s\n', opts.OutputDir);
fprintf('  target mean:  %.1f DN, max clip: dark %.2f%% / bright %.2f%%\n\n', ...
    opts.TargetMeanDN, opts.MaxDarkClipPct, opts.MaxBrightClipPct);

cam = webcam(webcamIdx);
cleanupObj = onCleanup(@() clearCamera(cam));
cam.Resolution = sprintf('%dx%d', opts.Resolution(1), opts.Resolution(2));

applyBaseSettings(cam, cfg, opts);
w = opts.Resolution(1);
h = opts.Resolution(2);
roi = sanitizeRoi(opts.Roi, w, h);
fprintf('Camera: %s\n', cam.Name);
fprintf('ROI: [%d %d %d %d]\n\n', roi);

expVals = double(opts.ExposureValues(:).');
gainVals = double(opts.GainValues(:).');
n = numel(expVals) * numel(gainVals);
rows = repmat(emptyRow(), n, 1);
idx = 0;

for e = expVals
    for g = gainVals
        idx = idx + 1;
        fprintf('[%2d/%2d] Exposure=%g Gain=%g\n', idx, n, e, g);
        rows(idx).requestedExposure = e;
        rows(idx).requestedGain = g;

        try
            setCameraProp(cam, 'ExposureMode', 'manual', 5, 0.3);
            setCameraProp(cam, 'Exposure', e, 5, 0.3);
            setCameraProp(cam, 'Gain', g, 5, 0.3);
            for k = 1:round(opts.WarmupFrames)
                safeSnapshot(cam, 4);
            end
            [metrics, representative] = captureMetrics(cam, roi, round(opts.FramesPerPair));

            rows(idx).actualExposure = getCameraProp(cam, 'Exposure');
            rows(idx).actualGain = getCameraProp(cam, 'Gain');
            rows(idx).meanDN = metrics.meanDN;
            rows(idx).p01DN = metrics.p01DN;
            rows(idx).p50DN = metrics.p50DN;
            rows(idx).p99DN = metrics.p99DN;
            rows(idx).darkClipPct = metrics.darkClipPct;
            rows(idx).brightClipPct = metrics.brightClipPct;
            rows(idx).tenengrad = metrics.tenengrad;
            rows(idx).lapVar = metrics.lapVar;
            rows(idx).temporalNoiseDN = metrics.temporalNoiseDN;
            rows(idx).score = scoreMetrics(metrics, opts);
            rows(idx).status = 'ok';

            imwrite(representative, fullfile(opts.OutputDir, ...
                sprintf('exp_%g_gain_%03g.png', e, g)));
            fprintf('  mean=%.1f p99=%.1f clip=%.2f%% score=%.3g\n', ...
                rows(idx).meanDN, rows(idx).p99DN, rows(idx).brightClipPct, rows(idx).score);
        catch ME
            rows(idx).status = ['failed: ' ME.message];
            rows(idx).score = -Inf;
            fprintf('  failed: %s\n', ME.message);
        end
    end
end

tbl = struct2table(rows);
csvFile = fullfile(opts.OutputDir, 'my8077_exposure_gain_metrics.csv');
writetable(tbl, csvFile);

valid = isfinite(tbl.score);
if any(valid)
    validIdx = find(valid);
    [~, localBest] = max(tbl.score(valid));
    bestIdx = validIdx(localBest);
else
    bestIdx = NaN;
end

result = struct();
result.outputDir = opts.OutputDir;
result.table = tbl;
result.roi = roi;
result.options = opts;
result.csvFile = csvFile;

if ~isnan(bestIdx)
    result.best = tbl(bestIdx,:);
    fprintf('\nRecommended:\n');
    fprintf('  Exposure = %g\n', result.best.requestedExposure);
    fprintf('  Gain     = %g\n', result.best.requestedGain);
    fprintf('  mean=%.1f DN, p99=%.1f DN, bright clip=%.2f%%, score=%.3g\n', ...
        result.best.meanDN, result.best.p99DN, result.best.brightClipPct, result.best.score);
else
    result.best = table();
    fprintf('\nNo valid candidate captured.\n');
end

save(fullfile(opts.OutputDir, 'my8077_exposure_gain_tune.mat'), 'result');
fprintf('\nSaved:\n  %s\n  %s\n', csvFile, fullfile(opts.OutputDir, 'my8077_exposure_gain_tune.mat'));

end


function applyBaseSettings(cam, cfg, opts)
setCameraProp(cam, 'FocusMode', 'manual', 3, 0.2);
setCameraProp(cam, 'Focus', cfg.cameraFocus, 3, 0.2);
setCameraProp(cam, 'ExposureMode', 'manual', 3, 0.2);
setCameraProp(cam, 'WhiteBalanceMode', 'manual', 3, 0.2);
setCameraProp(cam, 'BacklightCompensation', 0, 3, 0.2);
setCameraProp(cam, 'Brightness', opts.Brightness, 3, 0.2);
setCameraProp(cam, 'Contrast', opts.Contrast, 3, 0.2);
setCameraProp(cam, 'Gamma', opts.Gamma, 3, 0.2);
setCameraProp(cam, 'Saturation', opts.Saturation, 3, 0.2);
setCameraProp(cam, 'Sharpness', opts.Sharpness, 3, 0.2);
setCameraProp(cam, 'Zoom', 0, 3, 0.2);
setCameraProp(cam, 'Pan', 0, 3, 0.2);
setCameraProp(cam, 'Tilt', 0, 3, 0.2);
setCameraProp(cam, 'Roll', 3, 3, 0.2);
end


function [metrics, representative] = captureMetrics(cam, roi, nFrames)
frames = cell(1, nFrames);
grayStack = [];
for k = 1:nFrames
    frames{k} = safeSnapshot(cam, 6);
    gray = rgb2gray(frames{k});
    gray = gray(roi(2):(roi(2)+roi(4)-1), roi(1):(roi(1)+roi(3)-1));
    grayStack(:,:,k) = single(gray); %#ok<AGROW>
end
representative = frames{ceil(nFrames / 2)};
grayMean = mean(grayStack, 3);
vals = grayMean(:);

m = qualityMetrics(grayMean / 255);
pixelStd = std(grayStack, 0, 3);

metrics = struct();
metrics.meanDN = mean(vals);
metrics.p01DN = percentile(vals, 1);
metrics.p50DN = percentile(vals, 50);
metrics.p99DN = percentile(vals, 99);
metrics.darkClipPct = 100 * mean(vals <= 1);
metrics.brightClipPct = 100 * mean(vals >= 254);
metrics.tenengrad = m.tenengrad;
metrics.lapVar = m.lapVar;
metrics.temporalNoiseDN = mean(pixelStd(:));
end


function score = scoreMetrics(metrics, opts)
meanPenalty = abs(metrics.meanDN - opts.TargetMeanDN) / max(opts.TargetMeanDN, 1);
brightPenalty = max(0, metrics.brightClipPct - opts.MaxBrightClipPct) / 5;
darkPenalty = max(0, metrics.darkClipPct - opts.MaxDarkClipPct) / 5;
highlightHeadroomPenalty = max(0, metrics.p99DN - 245) / 30;
noisePenalty = metrics.temporalNoiseDN / 50;

detailScore = log1p(5000 * metrics.tenengrad);
score = detailScore ...
    - 1.2 * meanPenalty ...
    - 4.0 * brightPenalty ...
    - 2.0 * darkPenalty ...
    - 2.0 * highlightHeadroomPenalty ...
    - 0.5 * noisePenalty;
end


function row = emptyRow()
row = struct( ...
    'requestedExposure', NaN, ...
    'requestedGain', NaN, ...
    'actualExposure', NaN, ...
    'actualGain', NaN, ...
    'meanDN', NaN, ...
    'p01DN', NaN, ...
    'p50DN', NaN, ...
    'p99DN', NaN, ...
    'darkClipPct', NaN, ...
    'brightClipPct', NaN, ...
    'tenengrad', NaN, ...
    'lapVar', NaN, ...
    'temporalNoiseDN', NaN, ...
    'score', NaN, ...
    'status', 'not run');
end


function m = qualityMetrics(gray)
lapKernel = single([0 1 0; 1 -4 1; 0 1 0]);
sobelX = single([-1 0 1; -2 0 2; -1 0 1]) / 8;
sobelY = sobelX.';
lap = conv2(single(gray), lapKernel, 'valid');
gx = conv2(single(gray), sobelX, 'valid');
gy = conv2(single(gray), sobelY, 'valid');
m.lapVar = var(lap(:));
m.tenengrad = mean(gx(:).^2 + gy(:).^2);
end


function roi = sanitizeRoi(roiIn, frameW, frameH)
if isempty(roiIn)
    roi = [1 1 frameW frameH];
else
    roi = round(double(roiIn(:).'));
end
roi(1) = max(1, min(frameW, roi(1)));
roi(2) = max(1, min(frameH, roi(2)));
roi(3) = max(1, min(frameW - roi(1) + 1, roi(3)));
roi(4) = max(1, min(frameH - roi(2) + 1, roi(4)));
end


function frame = safeSnapshot(cam, maxTries)
lastErr = [];
for k = 1:maxTries
    try
        frame = snapshot(cam);
        return;
    catch ME
        lastErr = ME;
        pause(0.25);
    end
end
rethrow(lastErr);
end


function q = percentile(values, pct)
values = sort(double(values(:)));
values = values(isfinite(values));
if isempty(values)
    q = NaN;
    return;
end
idx = 1 + (numel(values) - 1) * pct / 100;
lo = max(1, floor(idx));
hi = min(numel(values), ceil(idx));
if lo == hi
    q = values(lo);
else
    q = values(lo) + (idx - lo) * (values(hi) - values(lo));
end
end


function ok = setCameraProp(cam, prop, val, maxTries, pauseSeconds)
if nargin < 4
    maxTries = 3;
end
if nargin < 5
    pauseSeconds = 0.2;
end
ok = false;
for k = 1:maxTries
    try
        cam.(prop) = val;
        pause(pauseSeconds);
        ok = true;
        return;
    catch
        pause(pauseSeconds);
    end
end
end


function trySet(cam, prop, val)
try
    cam.(prop) = val;
catch
end
end


function v = getCameraProp(cam, prop)
try
    v = cam.(prop);
catch
    v = NaN;
end
end


function s = toChar(v)
if ischar(v)
    s = v;
elseif isa(v, 'string')
    s = char(v);
else
    s = char(v);
end
end


function clearCamera(cam)
if ~isempty(cam)
    clear cam;
end
end
