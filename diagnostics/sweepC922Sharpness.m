function results = sweepC922Sharpness(logicalCam, sharpnessValues, varargin)
% SWEEPC922SHARPNESS  Capture a sharpness sweep for the configured C922 camera.
%
%   results = sweepC922Sharpness()
%   results = sweepC922Sharpness(logicalCam, sharpnessValues)
%   results = sweepC922Sharpness(..., 'Name', value)
%
%   Opens the camera from cfg.camIndices(logicalCam), applies the normal
%   buildConfig/applyCameraSettings path, then changes only the Sharpness
%   property. For each sharpness value it captures a short burst, saves a
%   representative frame, and prints/saves no-reference image-quality metrics.
%
%   Defaults target logical camera 2, i.e. cfg.camIndices(2), which is the
%   Logitech C922 in the current rig configuration.
%
%   Name-value options:
%     'FramesPerValue'  Number of measured frames per value, default 12.
%     'WarmupFrames'    Frames discarded after each sharpness change, default 5.
%     'OutputDir'       Output folder, default output/diagnostics/sharpness_*.
%     'Roi'             [x y w h] crop used for metrics, default full frame.
%     'SaveBurst'       Save every measured frame, default false.
%     'FreezeAuto'      Freeze exposure/WB after config settle, default true.
%
%   Example command-line use from the repo root:
%     matlab -batch "addpath(genpath('Program')); sweepC922Sharpness"
%
%   For a finer sweep:
%     matlab -batch "addpath(genpath('Program')); sweepC922Sharpness(2,0:8:128)"
%
%   Metric interpretation:
%     Higher lapVar/tenengrad usually means sharper edges, but can also mean
%     amplified noise. Lower temporalNoiseMean/temporalNoiseMedian is cleaner.
%     Use a static, detailed target; for noise, include a flat patch or pass Roi.
%
%   See also: buildConfig, applyCameraSettings

if nargin < 1 || isempty(logicalCam)
    logicalCam = 2;
end
if nargin < 2 || isempty(sharpnessValues)
    sharpnessValues = 0:16:255;
end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'FramesPerValue', 12, @(x) isnumeric(x) && isscalar(x) && x >= 2);
addParameter(p, 'WarmupFrames', 5, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'OutputDir', '', @(x) ischar(x) || isa(x, 'string'));
addParameter(p, 'Roi', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 4));
addParameter(p, 'SaveBurst', false, @(x) islogical(x) && isscalar(x));
addParameter(p, 'FreezeAuto', true, @(x) islogical(x) && isscalar(x));
parse(p, varargin{:});
opts = p.Results;

rootDir = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(genpath(rootDir));

cfg = buildConfig();
validateCameraSelection(cfg, logicalCam);

webcamIdx = cfg.camIndices(logicalCam);
opts.OutputDir = toChar(opts.OutputDir);
if isempty(opts.OutputDir)
    opts.OutputDir = fullfile(rootDir, cfg.logDir, 'diagnostics', ...
        ['sharpness_' datestr(now, 'yyyymmdd_HHMMSS')]);
end
if ~isfolder(opts.OutputDir)
    mkdir(opts.OutputDir);
end

fprintf('Sharpness sweep\n');
fprintf('  Logical camera: %d\n', logicalCam);
fprintf('  webcam index:   %d\n', webcamIdx);
fprintf('  Resolution:     %dx%d\n', cfg.resolution(1), cfg.resolution(2));
fprintf('  Output:         %s\n\n', opts.OutputDir);

cam = [];
cleanupObj = onCleanup(@() releaseCamera(cam));

cam = webcam(webcamIdx);
cam.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));

fprintf('Applying normal camera config...\n');
settled = applyCameraSettings(cam, cfg, 'full');

if opts.FreezeAuto
    trySet(cam, 'ExposureMode', 'manual');
    trySet(cam, 'WhiteBalanceMode', 'manual');
end

initial = readCameraState(cam);
writeCameraState(fullfile(opts.OutputDir, 'camera_state_initial.txt'), ...
    cam, cfg, settled, initial, opts);

if isnan(initial.Sharpness)
    error('sweepC922Sharpness:noSharpness', ...
          'Camera "%s" does not expose a Sharpness property through MATLAB.', cam.Name);
end

fprintf('Camera: %s\n', cam.Name);
fprintf('Initial state: Sharpness=%g Exposure=%g Gain=%g Focus=%g Zoom=%g\n', ...
    initial.Sharpness, initial.Exposure, initial.Gain, initial.Focus, initial.Zoom);
fprintf('Auto freeze: %d  ExposureMode=%s  WhiteBalanceMode=%s\n\n', ...
    opts.FreezeAuto, stateToString(initial.ExposureMode), stateToString(initial.WhiteBalanceMode));

sharpnessValues = double(sharpnessValues(:).');
nValues = numel(sharpnessValues);
nFrames = round(opts.FramesPerValue);
warmupFrames = round(opts.WarmupFrames);

rows = repmat(emptyResultRow(), nValues, 1);
representatives = cell(1, nValues);

for s = 1:nValues
    requestedSharpness = sharpnessValues(s);
    fprintf('[%2d/%2d] Sharpness = %g\n', s, nValues, requestedSharpness);

    try
        cam.Sharpness = requestedSharpness;
    catch ME
        warning('sweepC922Sharpness:setFailed', ...
            'Could not set Sharpness=%g: %s', requestedSharpness, ME.message);
        rows(s).requestedSharpness = requestedSharpness;
        rows(s).actualSharpness = NaN;
        continue;
    end

    for k = 1:warmupFrames
        snapshot(cam);
    end

    firstFrame = snapshot(cam);
    [h, w, ~] = size(firstFrame);
    roi = sanitizeRoi(opts.Roi, w, h);
    stack = zeros(roi(4), roi(3), nFrames, 'single');
    frameMeans = zeros(1, nFrames);
    lapVars = zeros(1, nFrames);
    tenengrads = zeros(1, nFrames);
    rmsContrasts = zeros(1, nFrames);
    darkClipPct = zeros(1, nFrames);
    brightClipPct = zeros(1, nFrames);
    frameTimesMs = zeros(1, nFrames);

    frames = cell(1, nFrames);
    frames{1} = firstFrame;
    for f = 1:nFrames
        if f > 1
            tFrame = tic;
            frames{f} = snapshot(cam);
            frameTimesMs(f) = toc(tFrame) * 1000;
        end

        gray = single(rgb2gray(frames{f})) / 255;
        gray = cropGray(gray, roi);
        stack(:,:,f) = gray;

        m = qualityMetrics(gray);
        frameMeans(f) = m.meanGray;
        lapVars(f) = m.lapVar;
        tenengrads(f) = m.tenengrad;
        rmsContrasts(f) = m.rmsContrast;
        darkClipPct(f) = m.darkClipPct;
        brightClipPct(f) = m.brightClipPct;

        if opts.SaveBurst
            imwrite(frames{f}, fullfile(opts.OutputDir, ...
                sprintf('sharpness_%03g_frame_%03d.png', requestedSharpness, f)));
        end
    end

    representative = frames{ceil(nFrames / 2)};
    representatives{s} = representative;
    imwrite(representative, fullfile(opts.OutputDir, ...
        sprintf('sharpness_%03g.png', requestedSharpness)));

    noise = temporalNoiseMetrics(stack);
    spatialNoise = spatialNoiseEstimate(stack(:,:,ceil(nFrames / 2)));
    state = readCameraState(cam);

    rows(s).requestedSharpness = requestedSharpness;
    rows(s).actualSharpness = state.Sharpness;
    rows(s).lapVar = meanFinite(lapVars);
    rows(s).tenengrad = meanFinite(tenengrads);
    rows(s).rmsContrast = meanFinite(rmsContrasts);
    rows(s).meanGray = meanFinite(frameMeans);
    rows(s).darkClipPct = meanFinite(darkClipPct);
    rows(s).brightClipPct = meanFinite(brightClipPct);
    rows(s).temporalNoiseMean = noise.meanStd255;
    rows(s).temporalNoiseMedian = noise.medianStd255;
    rows(s).frameMeanStd = stdFinite(frameMeans) * 255;
    rows(s).spatialNoiseMad = spatialNoise;
    rows(s).snapshotMs = meanFinite(frameTimesMs(2:end));
    rows(s).exposure = state.Exposure;
    rows(s).gain = state.Gain;
    rows(s).focus = state.Focus;
    rows(s).zoom = state.Zoom;
    rows(s).brightness = state.Brightness;
    rows(s).contrast = state.Contrast;
    rows(s).saturation = state.Saturation;
    rows(s).whiteBalance = state.WhiteBalance;

    fprintf(['  actual=%g | lapVar=%.4g | tenengrad=%.4g | ' ...
             'temporalNoise=%.2f/%.2f DN | mean=%.1f DN | clip=%.2f%%\n'], ...
        rows(s).actualSharpness, rows(s).lapVar, rows(s).tenengrad, ...
        rows(s).temporalNoiseMean, rows(s).temporalNoiseMedian, ...
        rows(s).meanGray * 255, rows(s).brightClipPct);
end

results = struct();
results.cfg = cfg;
results.logicalCam = logicalCam;
results.webcamIdx = webcamIdx;
results.cameraName = cam.Name;
results.outputDir = opts.OutputDir;
results.options = opts;
results.rows = rows;
results.table = struct2table(rows);

csvFile = fullfile(opts.OutputDir, 'sharpness_metrics.csv');
matFile = fullfile(opts.OutputDir, 'sharpness_metrics.mat');
writetable(results.table, csvFile);
save(matFile, 'results');

sheetFile = fullfile(opts.OutputDir, 'sharpness_contact_sheet.png');
writeContactSheet(representatives, sharpnessValues, sheetFile);

fprintf('\nSaved:\n');
fprintf('  %s\n', csvFile);
fprintf('  %s\n', matFile);
fprintf('  %s\n', sheetFile);
fprintf('\nBest by lapVar:    Sharpness=%g\n', bestSharpness(results.table, 'lapVar'));
fprintf('Best by tenengrad: Sharpness=%g\n', bestSharpness(results.table, 'tenengrad'));
fprintf('Lowest noise:      Sharpness=%g\n', bestSharpness(results.table, 'temporalNoiseMean', 'min'));

end


function validateCameraSelection(cfg, logicalCam)
if logicalCam < 1 || logicalCam > numel(cfg.camIndices)
    error('sweepC922Sharpness:badLogicalCam', ...
          'logicalCam=%d is outside cfg.camIndices, which has %d entries.', ...
          logicalCam, numel(cfg.camIndices));
end
end


function row = emptyResultRow()
row = struct( ...
    'requestedSharpness', NaN, ...
    'actualSharpness', NaN, ...
    'lapVar', NaN, ...
    'tenengrad', NaN, ...
    'rmsContrast', NaN, ...
    'meanGray', NaN, ...
    'darkClipPct', NaN, ...
    'brightClipPct', NaN, ...
    'temporalNoiseMean', NaN, ...
    'temporalNoiseMedian', NaN, ...
    'frameMeanStd', NaN, ...
    'spatialNoiseMad', NaN, ...
    'snapshotMs', NaN, ...
    'exposure', NaN, ...
    'gain', NaN, ...
    'focus', NaN, ...
    'zoom', NaN, ...
    'brightness', NaN, ...
    'contrast', NaN, ...
    'saturation', NaN, ...
    'whiteBalance', NaN);
end


function m = qualityMetrics(gray)
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


function n = temporalNoiseMetrics(stack)
pixelStd = std(stack, 0, 3);
n.meanStd255 = meanFinite(pixelStd(:)) * 255;
n.medianStd255 = medianFinite(pixelStd(:)) * 255;
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


function roi = sanitizeRoi(roiIn, frameW, frameH)
if isempty(roiIn)
    roi = [1 1 frameW frameH];
    return;
end
roi = round(double(roiIn(:).'));
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


function state = readCameraState(cam)
state = struct();
state.Name = cam.Name;
props = {'Sharpness','Exposure','ExposureMode','Gain','Focus','FocusMode', ...
         'Zoom','Brightness','Contrast','Saturation','WhiteBalance', ...
         'WhiteBalanceMode','BacklightCompensation','Pan','Tilt','Roll'};
for k = 1:numel(props)
    state.(props{k}) = tryGet(cam, props{k});
end
end


function writeCameraState(filename, cam, cfg, settled, state, opts)
fid = fopen(filename, 'w');
if fid < 0
    warning('sweepC922Sharpness:stateWriteFailed', ...
            'Could not write camera state file: %s', filename);
    return;
end
cleaner = onCleanup(@() fclose(fid));
fprintf(fid, 'Camera sharpness sweep initial state\n');
fprintf(fid, 'Camera: %s\n', cam.Name);
fprintf(fid, 'Resolution: %s\n', cam.Resolution);
fprintf(fid, 'cfg.cameraFocus: %g\n', cfg.cameraFocus);
fprintf(fid, 'settled.Exposure: %g\n', settled.Exposure);
fprintf(fid, 'settled.WhiteBalance: %g\n', settled.WhiteBalance);
fprintf(fid, 'settled.Focus: %g\n', settled.Focus);
fprintf(fid, 'FreezeAuto: %d\n', opts.FreezeAuto);
fprintf(fid, '\nProperties after config:\n');
fields = fieldnames(state);
for k = 1:numel(fields)
    val = state.(fields{k});
    if isnumeric(val)
        fprintf(fid, '%s: %g\n', fields{k}, val);
    elseif ischar(val) || isa(val, 'string')
        fprintf(fid, '%s: %s\n', fields{k}, char(val));
    else
        fprintf(fid, '%s: <unprintable>\n', fields{k});
    end
end
end


function writeContactSheet(frames, values, filename)
valid = ~cellfun(@isempty, frames);
frames = frames(valid);
values = values(valid);
if isempty(frames)
    return;
end

maxThumbW = 320;
[h, w, c] = size(frames{1});
scale = min(1, maxThumbW / w);
thumbH = max(1, round(h * scale));
thumbW = max(1, round(w * scale));
cols = min(4, numel(frames));
rows = ceil(numel(frames) / cols);
sheet = zeros(rows * thumbH, cols * thumbW, c, 'uint8');

for k = 1:numel(frames)
    r = floor((k - 1) / cols) + 1;
    col = mod(k - 1, cols) + 1;
    thumb = imresize(frames{k}, [thumbH thumbW]);
    rr = (r-1)*thumbH + (1:thumbH);
    cc = (col-1)*thumbW + (1:thumbW);
    sheet(rr, cc, :) = thumb;
end

imwrite(sheet, filename);

labelFile = regexprep(filename, '\.png$', '_labels.txt');
fid = fopen(labelFile, 'w');
if fid >= 0
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, 'Contact sheet labels, row-major order:\n');
    for k = 1:numel(values)
        fprintf(fid, '%d: Sharpness=%g\n', k, values(k));
    end
end
end


function value = bestSharpness(tbl, metricName, mode)
if nargin < 3
    mode = 'max';
end
metric = tbl.(metricName);
valid = isfinite(metric);
if ~any(valid)
    value = NaN;
    return;
end
switch mode
    case 'min'
        validIdx = find(valid);
        [~, localIdx] = min(metric(valid));
        idx = validIdx(localIdx);
    otherwise
        validIdx = find(valid);
        [~, localIdx] = max(metric(valid));
        idx = validIdx(localIdx);
end
value = tbl.requestedSharpness(idx);
end


function v = tryGet(cam, prop)
try
    v = cam.(prop);
catch
    v = NaN;
end
end


function trySet(cam, prop, val)
try
    cam.(prop) = val;
catch
end
end


function s = stateToString(v)
if ischar(v) || isa(v, 'string')
    s = char(v);
elseif isnumeric(v) && isnan(v)
    s = 'NaN';
else
    s = '<unprintable>';
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


function y = meanFinite(x)
x = x(isfinite(x));
if isempty(x)
    y = NaN;
else
    y = mean(x);
end
end


function y = stdFinite(x)
x = x(isfinite(x));
if numel(x) < 2
    y = NaN;
else
    y = std(x);
end
end


function y = varFinite(x)
x = x(isfinite(x));
if numel(x) < 2
    y = NaN;
else
    y = var(x);
end
end


function y = medianFinite(x)
x = x(isfinite(x));
if isempty(x)
    y = NaN;
else
    y = median(x);
end
end


function releaseCamera(cam)
if ~isempty(cam)
    clear cam;
end
end
