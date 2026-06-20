function results = runC922BlurDiagnostics(webcamIdx, varargin)
% RUNC922BLURDIAGNOSTICS  Run C922 blur/noise diagnostics from the command line.
%
%   results = runC922BlurDiagnostics()
%   results = runC922BlurDiagnostics(webcamIdx)
%   results = runC922BlurDiagnostics(..., 'Name', value)
%
%   The runner applies the normal buildConfig/applyCameraSettings path, freezes
%   exposure and white balance after settle, then runs these independent tests:
%     1. Focus sweep
%     2. Exposure sweep
%     3. Gain sweep
%     4. Resolution comparison
%     5. Zoom sweep
%     6. Spatial sharpness map
%
%   The default webcam index is 2 because the current temporary setup is:
%     1 = Integrated Camera, 2 = c922 Pro Stream Webcam
%
%   Name-value options:
%     'Roi'                [x y w h] crop for metrics at cfg resolution.
%     'FramesPerValue'     Measured frames per setting, default 10.
%     'WarmupFrames'       Frames discarded after each setting change, default 4.
%     'SharpnessForTests'  Fixed sharpness for non-sharpness tests, default 128.
%     'OutputDir'          Output folder. Defaults to output/diagnostics/blur_*.
%
%   Example:
%     matlab -batch "addpath(genpath('Program')); runC922BlurDiagnostics(2)"
%
%   See also: buildConfig, applyCameraSettings, sweepC922Sharpness

if nargin < 1 || isempty(webcamIdx)
    webcamIdx = 2;
end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'Roi', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 4));
addParameter(p, 'FramesPerValue', 10, @(x) isnumeric(x) && isscalar(x) && x >= 2);
addParameter(p, 'WarmupFrames', 4, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'SharpnessForTests', 128, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'OutputDir', '', @(x) ischar(x) || isa(x, 'string'));
parse(p, varargin{:});
opts = p.Results;
opts.OutputDir = toChar(opts.OutputDir);

rootDir = fullfile(fileparts(mfilename('fullpath')), '..');
addpath(genpath(rootDir));

cfg = buildConfig();
if isempty(opts.OutputDir)
    opts.OutputDir = fullfile(rootDir, cfg.logDir, 'diagnostics', ...
        ['blur_' datestr(now, 'yyyymmdd_HHMMSS')]);
end
if ~isfolder(opts.OutputDir)
    mkdir(opts.OutputDir);
end

fprintf('C922 blur diagnostics\n');
fprintf('  webcam index:   %d\n', webcamIdx);
fprintf('  cfg resolution: %dx%d\n', cfg.resolution(1), cfg.resolution(2));
fprintf('  output:         %s\n', opts.OutputDir);
fprintf('  metric sharpness for tests: %g\n\n', opts.SharpnessForTests);

results = struct();
results.cfg = cfg;
results.webcamIdx = webcamIdx;
results.options = opts;
results.outputDir = opts.OutputDir;

cam = openConfiguredCamera(webcamIdx, cfg, opts.SharpnessForTests);
cleanupObj = onCleanup(@() releaseCamera(cam));
baseState = readCameraState(cam);
results.cameraName = cam.Name;
results.baseState = baseState;

roi = sanitizeRoi(opts.Roi, cfg.resolution(1), cfg.resolution(2));
results.roi = roi;
writeInitialState(fullfile(opts.OutputDir, 'initial_state.txt'), cam, cfg, opts, baseState, roi);

fprintf('Camera: %s\n', cam.Name);
fprintf('Initial after config: Focus=%g Sharpness=%g Exposure=%g Gain=%g Zoom=%g\n\n', ...
    baseState.Focus, baseState.Sharpness, baseState.Exposure, baseState.Gain, baseState.Zoom);

% 1. Focus sweep.
focusDir = ensureSubdir(opts.OutputDir, 'focus_sweep');
focusValues = 0:10:250;
fprintf('--- Focus sweep (%d values) ---\n', numel(focusValues));
prepareBaseState(cam, cfg, opts.SharpnessForTests);
trySet(cam, 'FocusMode', 'manual');
focusRows = sweepControl(cam, 'Focus', focusValues, roi, focusDir, opts);
results.focus = struct();
results.focus.table = struct2table(focusRows);
results.focus.bestByTenengrad = bestRequested(results.focus.table, 'tenengrad', 'max');
results.focus.bestByLapVar = bestRequested(results.focus.table, 'lapVar', 'max');
writeTable(results.focus.table, fullfile(focusDir, 'focus_metrics.csv'));
fprintf('Best focus by tenengrad: %g\n\n', results.focus.bestByTenengrad);

% 2. Exposure sweep at current configured focus.
exposureDir = ensureSubdir(opts.OutputDir, 'exposure_sweep');
exposureValues = -11:-1;
fprintf('--- Exposure sweep (%d values) ---\n', numel(exposureValues));
prepareBaseState(cam, cfg, opts.SharpnessForTests);
trySet(cam, 'Focus', cfg.cameraFocus);
trySet(cam, 'ExposureMode', 'manual');
exposureRows = sweepControl(cam, 'Exposure', exposureValues, roi, exposureDir, opts);
results.exposure = struct();
results.exposure.table = struct2table(exposureRows);
writeTable(results.exposure.table, fullfile(exposureDir, 'exposure_metrics.csv'));
fprintf('Best exposure by tenengrad: %g\n\n', ...
    bestRequested(results.exposure.table, 'tenengrad', 'max'));

% 3. Gain sweep at the configured settled exposure.
gainDir = ensureSubdir(opts.OutputDir, 'gain_sweep');
gainValues = 0:16:255;
fprintf('--- Gain sweep (%d values) ---\n', numel(gainValues));
prepareBaseState(cam, cfg, opts.SharpnessForTests);
trySet(cam, 'Focus', cfg.cameraFocus);
trySet(cam, 'ExposureMode', 'manual');
trySet(cam, 'Exposure', baseState.Exposure);
gainRows = sweepControl(cam, 'Gain', gainValues, roi, gainDir, opts);
results.gain = struct();
results.gain.table = struct2table(gainRows);
writeTable(results.gain.table, fullfile(gainDir, 'gain_metrics.csv'));
fprintf('Lowest gain-sweep temporal noise: requested Gain=%g\n\n', ...
    bestRequested(results.gain.table, 'temporalNoiseMean', 'min'));

% 4. Resolution comparison.
resolutionDir = ensureSubdir(opts.OutputDir, 'resolution_compare');
fprintf('--- Resolution comparison ---\n');
resolutionRows = compareResolutions(cam, cfg, webcamIdx, roi, resolutionDir, opts);
results.resolution = struct();
results.resolution.table = struct2table(resolutionRows);
writeTable(results.resolution.table, fullfile(resolutionDir, 'resolution_metrics.csv'));
fprintf('Compared %d resolution(s).\n\n', numel(resolutionRows));

% Re-open at cfg resolution after resolution changes.
releaseCamera(cam);
cam = openConfiguredCamera(webcamIdx, cfg, opts.SharpnessForTests);

% 5. Zoom sweep.
zoomDir = ensureSubdir(opts.OutputDir, 'zoom_sweep');
zoomValues = [0 50 100 125 150 175 200 250 300 400 500];
fprintf('--- Zoom sweep (%d values) ---\n', numel(zoomValues));
prepareBaseState(cam, cfg, opts.SharpnessForTests);
trySet(cam, 'Focus', cfg.cameraFocus);
zoomRows = sweepControl(cam, 'Zoom', zoomValues, roi, zoomDir, opts);
results.zoom = struct();
results.zoom.table = struct2table(zoomRows);
writeTable(results.zoom.table, fullfile(zoomDir, 'zoom_metrics.csv'));
fprintf('Actual zoom values observed: %s\n\n', uniqueValuesString([zoomRows.actualValue]));

% 6. Spatial sharpness map.
mapDir = ensureSubdir(opts.OutputDir, 'spatial_map');
fprintf('--- Spatial sharpness map ---\n');
prepareBaseState(cam, cfg, opts.SharpnessForTests);
trySet(cam, 'Focus', cfg.cameraFocus);
map = spatialSharpnessMap(cam, mapDir, opts);
results.spatialMap = map;

summaryFile = fullfile(opts.OutputDir, 'summary.txt');
writeSummary(summaryFile, results);
save(fullfile(opts.OutputDir, 'blur_diagnostics.mat'), 'results');

fprintf('\nSaved summary:\n  %s\n', summaryFile);
fprintf('Saved MAT:\n  %s\n', fullfile(opts.OutputDir, 'blur_diagnostics.mat'));
fprintf('Done.\n');

end


function cam = openConfiguredCamera(webcamIdx, cfg, sharpness)
cam = webcam(webcamIdx);
cam.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));
applyCameraSettings(cam, cfg, 'full');
trySet(cam, 'ExposureMode', 'manual');
trySet(cam, 'WhiteBalanceMode', 'manual');
trySet(cam, 'Sharpness', sharpness);
trySet(cam, 'FocusMode', 'manual');
trySet(cam, 'Focus', cfg.cameraFocus);
end


function prepareBaseState(cam, cfg, sharpness)
trySet(cam, 'ExposureMode', 'manual');
trySet(cam, 'WhiteBalanceMode', 'manual');
trySet(cam, 'FocusMode', 'manual');
trySet(cam, 'Focus', cfg.cameraFocus);
trySet(cam, 'Sharpness', sharpness);
end


function rows = sweepControl(cam, controlName, values, roi, outDir, opts)
values = double(values(:).');
rows = repmat(emptyRow(controlName), numel(values), 1);
for k = 1:numel(values)
    requested = values(k);
    rows(k).control = controlName;
    rows(k).requestedValue = requested;
    fprintf('  %s = %g\n', controlName, requested);

    ok = true;
    try
        cam.(controlName) = requested;
    catch ME
        ok = false;
        rows(k).status = ['set failed: ' ME.message];
        warning('runC922BlurDiagnostics:setFailed', ...
            'Could not set %s=%g: %s', controlName, requested, ME.message);
    end
    if ~ok
        continue;
    end

    try
        for w = 1:round(opts.WarmupFrames)
            safeSnapshot(cam, 3);
        end
        [metrics, representative] = captureMetrics(cam, roi, round(opts.FramesPerValue));
    catch ME
        rows(k).status = ['capture failed: ' ME.message];
        warning('runC922BlurDiagnostics:captureFailed', ...
            'Could not capture after setting %s=%g: %s', ...
            controlName, requested, ME.message);
        continue;
    end
    state = readCameraState(cam);
    actual = tryGet(state, controlName);

    rows(k) = fillRow(rows(k), requested, actual, metrics, state, 'ok');
    imwrite(representative, fullfile(outDir, ...
        sprintf('%s_%s.png', lower(controlName), valueToken(actual))));

    fprintf('    actual=%g | tenengrad=%.4g | lapVar=%.4g | noise=%.2f DN | mean=%.1f DN | clip=%.2f%%\n', ...
        rows(k).actualValue, rows(k).tenengrad, rows(k).lapVar, ...
        rows(k).temporalNoiseMean, rows(k).meanGray * 255, rows(k).brightClipPct);
end
end


function rows = compareResolutions(cam, cfg, webcamIdx, baseRoi, outDir, opts)
available = availableResolutions(cam);
preferred = {'640x480','800x600','960x540','1280x720','1920x1080'};
resList = {};
for k = 1:numel(preferred)
    if any(strcmp(available, preferred{k}))
        resList{end+1} = preferred{k}; %#ok<AGROW>
    end
end
if isempty(resList)
    resList = available;
end

rows = repmat(emptyResolutionRow(), numel(resList), 1);
baseW = cfg.resolution(1);
baseH = cfg.resolution(2);

for k = 1:numel(resList)
    res = resList{k};
    fprintf('  Resolution = %s\n', res);

    ok = true;
    try
        cam.Resolution = res;
    catch
        releaseCamera(cam);
        cam = webcam(webcamIdx);
        try
            cam.Resolution = res;
        catch ME
            ok = false;
            rows(k).resolution = res;
            rows(k).status = ['set failed: ' ME.message];
            warning('runC922BlurDiagnostics:resolutionFailed', ...
                'Could not set resolution %s: %s', res, ME.message);
        end
    end
    if ~ok
        continue;
    end

    wh = parseResolution(cam.Resolution);
    cfgRes = cfg;
    cfgRes.resolution = wh;
    applyCameraSettings(cam, cfgRes, 'full');
    trySet(cam, 'ExposureMode', 'manual');
    trySet(cam, 'WhiteBalanceMode', 'manual');
    trySet(cam, 'Sharpness', opts.SharpnessForTests);
    trySet(cam, 'FocusMode', 'manual');
    trySet(cam, 'Focus', cfg.cameraFocus);

    scaledRoi = scaleRoi(baseRoi, baseW, baseH, wh(1), wh(2));
    [metrics, representative] = captureMetrics(cam, scaledRoi, round(opts.FramesPerValue));
    state = readCameraState(cam);

    rows(k).resolution = cam.Resolution;
    rows(k).width = wh(1);
    rows(k).height = wh(2);
    rows(k).roiX = scaledRoi(1);
    rows(k).roiY = scaledRoi(2);
    rows(k).roiW = scaledRoi(3);
    rows(k).roiH = scaledRoi(4);
    rows(k).status = 'ok';
    rows(k).lapVar = metrics.lapVar;
    rows(k).tenengrad = metrics.tenengrad;
    rows(k).rmsContrast = metrics.rmsContrast;
    rows(k).meanGray = metrics.meanGray;
    rows(k).darkClipPct = metrics.darkClipPct;
    rows(k).brightClipPct = metrics.brightClipPct;
    rows(k).temporalNoiseMean = metrics.temporalNoiseMean;
    rows(k).temporalNoiseMedian = metrics.temporalNoiseMedian;
    rows(k).spatialNoiseMad = metrics.spatialNoiseMad;
    rows(k).exposure = state.Exposure;
    rows(k).gain = state.Gain;
    rows(k).focus = state.Focus;
    rows(k).sharpness = state.Sharpness;
    rows(k).zoom = state.Zoom;

    imwrite(representative, fullfile(outDir, ['resolution_' safeFileToken(cam.Resolution) '.png']));
    fprintf('    tenengrad=%.4g | lapVar=%.4g | noise=%.2f DN | exposure=%g gain=%g\n', ...
        rows(k).tenengrad, rows(k).lapVar, rows(k).temporalNoiseMean, ...
        rows(k).exposure, rows(k).gain);
end
end


function map = spatialSharpnessMap(cam, outDir, opts)
for k = 1:round(opts.WarmupFrames)
    safeSnapshot(cam, 3);
end
frame = safeSnapshot(cam, 5);
gray = single(rgb2gray(frame)) / 255;

nRows = 6;
nCols = 8;
[h, w] = size(gray);
tileH = floor(h / nRows);
tileW = floor(w / nCols);
lapVar = zeros(nRows, nCols);
tenengrad = zeros(nRows, nCols);

for r = 1:nRows
    for c = 1:nCols
        y1 = (r - 1) * tileH + 1;
        x1 = (c - 1) * tileW + 1;
        if r == nRows, y2 = h; else, y2 = r * tileH; end
        if c == nCols, x2 = w; else, x2 = c * tileW; end
        m = basicQualityMetrics(gray(y1:y2, x1:x2));
        lapVar(r,c) = m.lapVar;
        tenengrad(r,c) = m.tenengrad;
    end
end

map = struct();
map.lapVar = lapVar;
map.tenengrad = tenengrad;
map.frameFile = fullfile(outDir, 'spatial_map_frame.png');
map.lapHeatmapFile = fullfile(outDir, 'lapvar_heatmap.png');
map.tenengradHeatmapFile = fullfile(outDir, 'tenengrad_heatmap.png');
map.lapCsvFile = fullfile(outDir, 'lapvar_tiles.csv');
map.tenengradCsvFile = fullfile(outDir, 'tenengrad_tiles.csv');

imwrite(frame, map.frameFile);
csvwrite(map.lapCsvFile, lapVar);
csvwrite(map.tenengradCsvFile, tenengrad);
writeHeatmap(lapVar, map.lapHeatmapFile, 'Laplacian variance by tile');
writeHeatmap(tenengrad, map.tenengradHeatmapFile, 'Tenengrad by tile');

fprintf('  Spatial map saved.\n');
end


function [metrics, representative] = captureMetrics(cam, roi, nFrames)
frames = cell(1, nFrames);
firstFrame = safeSnapshot(cam, 5);
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
        frames{f} = safeSnapshot(cam, 5);
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


function row = emptyRow(controlName)
row = struct( ...
    'control', controlName, ...
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
    'whiteBalance', NaN);
end


function row = fillRow(row, requested, actual, metrics, state, status)
row.requestedValue = requested;
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
row.whiteBalance = state.WhiteBalance;
end


function row = emptyResolutionRow()
row = struct( ...
    'resolution', '', ...
    'width', NaN, ...
    'height', NaN, ...
    'roiX', NaN, ...
    'roiY', NaN, ...
    'roiW', NaN, ...
    'roiH', NaN, ...
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
    'zoom', NaN);
end


function state = readCameraState(cam)
state = struct();
state.Name = cam.Name;
state.Resolution = cam.Resolution;
props = {'Sharpness','Exposure','ExposureMode','Gain','Focus','FocusMode', ...
         'Zoom','Brightness','Contrast','Saturation','WhiteBalance', ...
         'WhiteBalanceMode','BacklightCompensation','Pan','Tilt','Roll'};
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


function frame = safeSnapshot(cam, maxTries)
if nargin < 2
    maxTries = 3;
end
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
if isempty(lastErr)
    error('runC922BlurDiagnostics:snapshotFailed', ...
          'Snapshot failed for an unknown reason.');
else
    rethrow(lastErr);
end
end


function v = tryGet(s, prop)
if isstruct(s) && isfield(s, prop)
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


function list = availableResolutions(cam)
try
    list = cam.AvailableResolutions;
catch
    list = {cam.Resolution};
end
if ischar(list)
    list = cellstr(list);
end
end


function wh = parseResolution(res)
parts = sscanf(char(res), '%dx%d');
if numel(parts) ~= 2
    wh = [NaN NaN];
else
    wh = parts(:).';
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


function roi = scaleRoi(baseRoi, baseW, baseH, frameW, frameH)
roi = round([ ...
    baseRoi(1) * frameW / baseW, ...
    baseRoi(2) * frameH / baseH, ...
    baseRoi(3) * frameW / baseW, ...
    baseRoi(4) * frameH / baseH]);
roi = sanitizeRoi(roi, frameW, frameH);
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


function outDir = ensureSubdir(parent, name)
outDir = fullfile(parent, name);
if ~isfolder(outDir)
    mkdir(outDir);
end
end


function writeTable(tbl, filename)
writetable(tbl, filename);
end


function token = valueToken(v)
if ~isfinite(v)
    token = 'NaN';
else
    token = regexprep(sprintf('%g', v), '[^\w-]', '_');
end
end


function token = safeFileToken(s)
token = regexprep(char(s), '[^\w-]', '_');
end


function s = uniqueValuesString(values)
values = unique(values(isfinite(values)));
if isempty(values)
    s = 'none';
else
    parts = arrayfun(@(v) sprintf('%g', v), values, 'UniformOutput', false);
    s = strjoin(parts, ', ');
end
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
if ismember('requestedValue', tbl.Properties.VariableNames)
    value = tbl.requestedValue(idx);
else
    value = idx;
end
end


function writeHeatmap(values, filename, titleText)
fig = figure('Visible', 'off');
imagesc(values);
axis image;
colorbar;
title(titleText);
xlabel('tile column');
ylabel('tile row');
saveas(fig, filename);
close(fig);
end


function writeInitialState(filename, cam, cfg, opts, state, roi)
fid = fopen(filename, 'w');
if fid < 0
    return;
end
cleaner = onCleanup(@() fclose(fid));
fprintf(fid, 'C922 blur diagnostics initial state\n');
fprintf(fid, 'Camera: %s\n', cam.Name);
fprintf(fid, 'Resolution: %s\n', cam.Resolution);
fprintf(fid, 'cfg.resolution: %dx%d\n', cfg.resolution(1), cfg.resolution(2));
fprintf(fid, 'cfg.cameraFocus: %g\n', cfg.cameraFocus);
fprintf(fid, 'SharpnessForTests: %g\n', opts.SharpnessForTests);
fprintf(fid, 'Roi: [%d %d %d %d]\n\n', roi);
fields = fieldnames(state);
for k = 1:numel(fields)
    val = state.(fields{k});
    if isnumeric(val)
        fprintf(fid, '%s: %g\n', fields{k}, val);
    elseif ischar(val) || isa(val, 'string')
        fprintf(fid, '%s: %s\n', fields{k}, char(val));
    end
end
end


function writeSummary(filename, results)
fid = fopen(filename, 'w');
if fid < 0
    return;
end
cleaner = onCleanup(@() fclose(fid));
fprintf(fid, 'C922 blur diagnostics summary\n');
fprintf(fid, 'Camera: %s\n', results.cameraName);
fprintf(fid, 'Output: %s\n', results.outputDir);
fprintf(fid, 'ROI: [%d %d %d %d]\n\n', results.roi);

fprintf(fid, 'Focus sweep:\n');
fprintf(fid, '  Best requested focus by Tenengrad: %g\n', results.focus.bestByTenengrad);
fprintf(fid, '  Best requested focus by LapVar:    %g\n\n', results.focus.bestByLapVar);

fprintf(fid, 'Exposure sweep:\n');
printBest(fid, results.exposure.table, 'tenengrad', 'max', '  Best Tenengrad');
printBest(fid, results.exposure.table, 'temporalNoiseMean', 'min', '  Lowest temporal noise');
fprintf(fid, '\n');

fprintf(fid, 'Gain sweep:\n');
printBest(fid, results.gain.table, 'tenengrad', 'max', '  Best Tenengrad');
printBest(fid, results.gain.table, 'temporalNoiseMean', 'min', '  Lowest temporal noise');
fprintf(fid, '\n');

fprintf(fid, 'Zoom sweep:\n');
fprintf(fid, '  Actual zoom values observed: %s\n\n', ...
    uniqueValuesString(results.zoom.table.actualValue));

fprintf(fid, 'Resolution comparison:\n');
for k = 1:height(results.resolution.table)
    r = results.resolution.table(k,:);
    fprintf(fid, '  %s: tenengrad %.4g, lapVar %.4g, noise %.2f DN, exposure %g, gain %g\n', ...
        char(r.resolution), r.tenengrad, r.lapVar, r.temporalNoiseMean, r.exposure, r.gain);
end
end


function printBest(fid, tbl, metricName, mode, label)
metric = tbl.(metricName);
valid = isfinite(metric);
if ~any(valid)
    fprintf(fid, '%s: none\n', label);
    return;
end
validIdx = find(valid);
switch mode
    case 'min'
        [bestVal, localIdx] = min(metric(valid));
    otherwise
        [bestVal, localIdx] = max(metric(valid));
end
idx = validIdx(localIdx);
fprintf(fid, '%s: requested %g, actual %g, metric %.4g\n', ...
    label, tbl.requestedValue(idx), tbl.actualValue(idx), bestVal);
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


function releaseCamera(cam)
if ~isempty(cam)
    clear cam;
end
end
