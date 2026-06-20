function results = sweepC922Control(webcamIdx, controlName, values, varargin)
% SWEEPC922CONTROL  Robust one-control C922 sweep for blur/noise diagnostics.
%
%   results = sweepC922Control(webcamIdx, controlName, values)
%   results = sweepC922Control(..., 'Name', value)
%
%   Opens the C922, applies the normal buildConfig/applyCameraSettings path,
%   freezes exposure/WB, sets a fixed sharpness, then sweeps one camera
%   property. Designed to be run in small command-line chunks because this
%   webcam/driver combination sometimes times out.
%
%   Example:
%     matlab -batch "addpath(genpath('Program')); sweepC922Control(2,'Focus',0:10:250)"
%
%   Options:
%     'Roi'                [x y w h] metric crop, default center crop.
%     'FramesPerValue'     Captured frames per setting, default 6.
%     'WarmupFrames'       Frames discarded after setting change, default 2.
%     'OutputDir'          Output folder, default output/diagnostics/<control>_*.
%     'SharpnessForTests'  Fixed sharpness for non-sharpness sweeps, default 128.
%     'FixedExposure'      Manual exposure to hold, default [].
%     'FixedGain'          Manual gain to hold, default [].
%     'FixedFocus'         Manual focus to hold, default cfg.cameraFocus.
%     'Resolution'         Camera resolution, default cfg.resolution.
%     'ReopenPerValue'     Reopen camera for each value, default false.
%
%   See also: runC922BlurDiagnostics, sweepC922Sharpness

if nargin < 1 || isempty(webcamIdx)
    webcamIdx = 2;
end
if nargin < 2 || isempty(controlName)
    controlName = 'Focus';
end
if nargin < 3 || isempty(values)
    values = 0:10:250;
end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'Roi', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 4));
addParameter(p, 'FramesPerValue', 6, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(p, 'WarmupFrames', 2, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'OutputDir', '', @(x) ischar(x) || isa(x, 'string'));
addParameter(p, 'SharpnessForTests', 128, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'FixedExposure', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'FixedGain', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'FixedFocus', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
addParameter(p, 'Resolution', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 2));
addParameter(p, 'ReopenPerValue', false, @(x) islogical(x) && isscalar(x));
parse(p, varargin{:});
opts = p.Results;
opts.OutputDir = toChar(opts.OutputDir);

rootDir = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(genpath(rootDir));
cfg = buildConfig();
if isempty(opts.Resolution)
    opts.Resolution = cfg.resolution;
end
if isempty(opts.OutputDir)
    opts.OutputDir = fullfile(rootDir, cfg.logDir, 'diagnostics', ...
        [lower(controlName) '_' datestr(now, 'yyyymmdd_HHMMSS')]);
end
if ~isfolder(opts.OutputDir)
    mkdir(opts.OutputDir);
end

values = double(values(:).');
roi = sanitizeRoi(opts.Roi, opts.Resolution(1), opts.Resolution(2));
rows = repmat(emptyRow(), numel(values), 1);

fprintf('%s sweep on webcam %d\n', controlName, webcamIdx);
fprintf('  resolution: %dx%d\n', opts.Resolution(1), opts.Resolution(2));
fprintf('  output:     %s\n', opts.OutputDir);
fprintf('  ROI:        [%d %d %d %d]\n\n', roi);

cam = [];
if ~opts.ReopenPerValue
    cam = openConfiguredCamera(webcamIdx, cfg, opts);
    cleanupObj = onCleanup(@() clearCamera(cam));
end

for k = 1:numel(values)
    requested = values(k);
    fprintf('[%2d/%2d] %s = %g\n', k, numel(values), controlName, requested);

    if opts.ReopenPerValue
        cam = openConfiguredCamera(webcamIdx, cfg, opts);
    end

    rows(k).requestedValue = requested;
    try
        cam.(controlName) = requested;
    catch ME
        rows(k).status = ['set failed: ' ME.message];
        fprintf('  set failed: %s\n', ME.message);
        if opts.ReopenPerValue, clear cam; end
        continue;
    end

    try
        for w = 1:round(opts.WarmupFrames)
            safeSnapshot(cam, 4);
        end
        [metrics, representative] = captureMetrics(cam, roi, round(opts.FramesPerValue));
        state = readCameraState(cam);
        actual = getStructValue(state, controlName);

        rows(k) = fillRow(rows(k), actual, metrics, state, 'ok');
        imageFile = fullfile(opts.OutputDir, sprintf('%s_%s.png', ...
            lower(controlName), valueToken(actual)));
        imwrite(representative, imageFile);

        fprintf('  actual=%g | tenengrad=%.4g | lapVar=%.4g | noise=%.2f DN | mean=%.1f DN | clip=%.2f%%\n', ...
            rows(k).actualValue, rows(k).tenengrad, rows(k).lapVar, ...
            rows(k).temporalNoiseMean, rows(k).meanGray * 255, rows(k).brightClipPct);
    catch ME
        rows(k).status = ['capture failed: ' ME.message];
        fprintf('  capture failed: %s\n', ME.message);
        if ~opts.ReopenPerValue
            try
                clear cam;
            catch
            end
            cam = openConfiguredCamera(webcamIdx, cfg, opts);
        end
    end

    if opts.ReopenPerValue
        clear cam;
    end
end

tbl = struct2table(rows);
csvFile = fullfile(opts.OutputDir, [lower(controlName) '_metrics.csv']);
matFile = fullfile(opts.OutputDir, [lower(controlName) '_metrics.mat']);
writetable(tbl, csvFile);

results = struct();
results.webcamIdx = webcamIdx;
results.controlName = controlName;
results.values = values;
results.options = opts;
results.outputDir = opts.OutputDir;
results.roi = roi;
results.table = tbl;
results.bestTenengrad = bestRequested(tbl, 'tenengrad', 'max');
results.bestLapVar = bestRequested(tbl, 'lapVar', 'max');
results.lowestNoise = bestRequested(tbl, 'temporalNoiseMean', 'min');
save(matFile, 'results');

fprintf('\nSaved:\n  %s\n  %s\n', csvFile, matFile);
fprintf('Best by tenengrad: %g\n', results.bestTenengrad);
fprintf('Best by lapVar:    %g\n', results.bestLapVar);
fprintf('Lowest noise:      %g\n', results.lowestNoise);

end


function cam = openConfiguredCamera(webcamIdx, cfg, opts)
cam = webcam(webcamIdx);
cam.Resolution = sprintf('%dx%d', opts.Resolution(1), opts.Resolution(2));
applyCameraSettings(cam, cfg, 'full');
trySet(cam, 'ExposureMode', 'manual');
trySet(cam, 'WhiteBalanceMode', 'manual');
trySet(cam, 'FocusMode', 'manual');
if isempty(opts.FixedFocus)
    trySet(cam, 'Focus', cfg.cameraFocus);
else
    trySet(cam, 'Focus', opts.FixedFocus);
end
trySet(cam, 'Sharpness', opts.SharpnessForTests);
if ~isempty(opts.FixedExposure)
    trySet(cam, 'ExposureMode', 'manual');
    trySet(cam, 'Exposure', opts.FixedExposure);
end
if ~isempty(opts.FixedGain)
    trySet(cam, 'Gain', opts.FixedGain);
end
end


function [metrics, representative] = captureMetrics(cam, roi, nFrames)
frames = cell(1, nFrames);
firstFrame = safeSnapshot(cam, 6);
[h, w, ~] = size(firstFrame);
roi = sanitizeRoi(roi, w, h);
stack = zeros(roi(4), roi(3), nFrames, 'single');
frameMeans = zeros(1, nFrames);
lapVars = zeros(1, nFrames);
tenengrads = zeros(1, nFrames);
rmsContrasts = zeros(1, nFrames);
darkClipPct = zeros(1, nFrames);
brightClipPct = zeros(1, nFrames);

for f = 1:nFrames
    if f == 1
        frames{f} = firstFrame;
    else
        frames{f} = safeSnapshot(cam, 6);
    end
    gray = single(rgb2gray(frames{f})) / 255;
    gray = cropGray(gray, roi);
    stack(:,:,f) = gray;

    m = basicQualityMetrics(gray);
    frameMeans(f) = m.meanGray;
    lapVars(f) = m.lapVar;
    tenengrads(f) = m.tenengrad;
    rmsContrasts(f) = m.rmsContrast;
    darkClipPct(f) = m.darkClipPct;
    brightClipPct(f) = m.brightClipPct;
end

representative = frames{ceil(nFrames / 2)};
pixelStd = std(stack, 0, 3);

metrics = struct();
metrics.meanGray = meanFinite(frameMeans);
metrics.rmsContrast = meanFinite(rmsContrasts);
metrics.lapVar = meanFinite(lapVars);
metrics.tenengrad = meanFinite(tenengrads);
metrics.darkClipPct = meanFinite(darkClipPct);
metrics.brightClipPct = meanFinite(brightClipPct);
metrics.temporalNoiseMean = meanFinite(pixelStd(:)) * 255;
metrics.temporalNoiseMedian = medianFinite(pixelStd(:)) * 255;
metrics.spatialNoiseMad = spatialNoiseEstimate(stack(:,:,ceil(nFrames / 2)));
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


function m = basicQualityMetrics(gray)
lapKernel = single([0 1 0; 1 -4 1; 0 1 0]);
sobelX = single([-1 0 1; -2 0 2; -1 0 1]) / 8;
sobelY = sobelX.';
lap = conv2(gray, lapKernel, 'valid');
gx = conv2(gray, sobelX, 'valid');
gy = conv2(gray, sobelY, 'valid');
grad2 = gx.^2 + gy.^2;
m.meanGray = meanFinite(gray(:));
m.rmsContrast = stdFinite(gray(:));
m.lapVar = varFinite(lap(:));
m.tenengrad = meanFinite(grad2(:));
m.darkClipPct = 100 * mean(gray(:) <= 1/255);
m.brightClipPct = 100 * mean(gray(:) >= 254/255);
end


function row = emptyRow()
row = struct( ...
    'requestedValue', NaN, ...
    'actualValue', NaN, ...
    'status', 'not run', ...
    'lapVar', NaN, ...
    'tenengrad', NaN, ...
    'rmsContrast', NaN, ...
    'meanGray', NaN, ...
    'darkClipPct', NaN, ...
    'brightClipPct', NaN, ...
    'temporalNoiseMean', NaN, ...
    'temporalNoiseMedian', NaN, ...
    'spatialNoiseMad', NaN, ...
    'exposure', NaN, ...
    'gain', NaN, ...
    'focus', NaN, ...
    'sharpness', NaN, ...
    'zoom', NaN, ...
    'brightness', NaN, ...
    'contrast', NaN, ...
    'saturation', NaN, ...
    'gamma', NaN, ...
    'whiteBalance', NaN);
end


function row = fillRow(row, actual, metrics, state, status)
row.actualValue = actual;
row.status = status;
row.lapVar = metrics.lapVar;
row.tenengrad = metrics.tenengrad;
row.rmsContrast = metrics.rmsContrast;
row.meanGray = metrics.meanGray;
row.darkClipPct = metrics.darkClipPct;
row.brightClipPct = metrics.brightClipPct;
row.temporalNoiseMean = metrics.temporalNoiseMean;
row.temporalNoiseMedian = metrics.temporalNoiseMedian;
row.spatialNoiseMad = metrics.spatialNoiseMad;
row.exposure = state.Exposure;
row.gain = state.Gain;
row.focus = state.Focus;
row.sharpness = state.Sharpness;
row.zoom = state.Zoom;
row.brightness = state.Brightness;
row.contrast = state.Contrast;
row.saturation = state.Saturation;
row.gamma = state.Gamma;
row.whiteBalance = state.WhiteBalance;
end


function state = readCameraState(cam)
state = struct();
props = {'Sharpness','Exposure','Gain','Focus','Zoom','Brightness', ...
         'Contrast','Saturation','Gamma','WhiteBalance'};
for k = 1:numel(props)
    state.(props{k}) = getCameraProp(cam, props{k});
end
end


function v = getCameraProp(cam, prop)
try
    v = cam.(prop);
catch
    v = NaN;
end
end


function v = getStructValue(s, prop)
if isfield(s, prop)
    v = s.(prop);
else
    v = NaN;
end
end


function trySet(cam, prop, val)
try
    cam.(prop) = val;
catch
end
end


function roi = sanitizeRoi(roiIn, frameW, frameH)
if isempty(roiIn)
    roiW = round(frameW * 0.45);
    roiH = round(frameH * 0.45);
    roi = [round((frameW - roiW) / 2), round((frameH - roiH) / 2), roiW, roiH];
else
    roi = round(double(roiIn(:).'));
end
roi(1) = max(1, min(frameW, roi(1)));
roi(2) = max(1, min(frameH, roi(2)));
roi(3) = max(1, min(frameW - roi(1) + 1, roi(3)));
roi(4) = max(1, min(frameH - roi(2) + 1, roi(4)));
end


function gray = cropGray(gray, roi)
x = roi(1);
y = roi(2);
w = roi(3);
h = roi(4);
gray = gray(y:(y+h-1), x:(x+w-1));
end


function noise255 = spatialNoiseEstimate(gray)
sobelX = single([-1 0 1; -2 0 2; -1 0 1]) / 8;
sobelY = sobelX.';
gx = conv2(gray, sobelX, 'same');
gy = conv2(gray, sobelY, 'same');
grad = sqrt(gx.^2 + gy.^2);
flatThr = percentile(grad(:), 25);
flatMask = grad <= flatThr;
smooth = conv2(gray, ones(3, 'single') / 9, 'same');
resid = gray - smooth;
vals = resid(flatMask);
vals = vals(isfinite(vals));
if isempty(vals)
    noise255 = NaN;
else
    medVal = median(vals);
    noise255 = 1.4826 * median(abs(vals - medVal)) * 255;
end
end


function q = percentile(values, pct)
values = values(isfinite(values));
if isempty(values)
    q = NaN;
    return;
end
values = sort(values(:));
idx = 1 + (numel(values) - 1) * pct / 100;
lo = max(1, floor(idx));
hi = min(numel(values), ceil(idx));
if lo == hi
    q = values(lo);
else
    q = values(lo) + (idx - lo) * (values(hi) - values(lo));
end
end


function y = meanFinite(x)
x = x(isfinite(x));
if isempty(x), y = NaN; else, y = mean(x); end
end


function y = stdFinite(x)
x = x(isfinite(x));
if numel(x) < 2, y = NaN; else, y = std(x); end
end


function y = varFinite(x)
x = x(isfinite(x));
if numel(x) < 2, y = NaN; else, y = var(x); end
end


function y = medianFinite(x)
x = x(isfinite(x));
if isempty(x), y = NaN; else, y = median(x); end
end


function value = bestRequested(tbl, metricName, mode)
metric = tbl.(metricName);
valid = isfinite(metric);
if ~any(valid)
    value = NaN;
    return;
end
validIdx = find(valid);
switch mode
    case 'min'
        [~, localIdx] = min(metric(valid));
    otherwise
        [~, localIdx] = max(metric(valid));
end
idx = validIdx(localIdx);
value = tbl.requestedValue(idx);
end


function token = valueToken(v)
if ~isfinite(v)
    token = 'NaN';
else
    token = regexprep(sprintf('%g', v), '[^\w-]', '_');
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
