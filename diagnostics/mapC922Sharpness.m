function result = mapC922Sharpness(webcamIdx, varargin)
% MAPC922SHARPNESS  Tile-wise sharpness map for the Logitech C922.
%
%   result = mapC922Sharpness()
%   result = mapC922Sharpness(webcamIdx, 'Name', value)
%
%   Captures one frame at fixed settings and writes tile-wise Laplacian
%   variance / Tenengrad heatmaps. Use this to diagnose asymmetric blur,
%   corner softness, smudges, or lens decentering.

if nargin < 1 || isempty(webcamIdx)
    webcamIdx = 2;
end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'Resolution', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 2));
addParameter(p, 'Sharpness', 128, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Focus', 0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Exposure', -5, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Gain', 0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Rows', 6, @(x) isnumeric(x) && isscalar(x) && x >= 2);
addParameter(p, 'Cols', 8, @(x) isnumeric(x) && isscalar(x) && x >= 2);
addParameter(p, 'WarmupFrames', 5, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'OutputDir', '', @(x) ischar(x) || isa(x, 'string'));
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
        ['spatial_' datestr(now, 'yyyymmdd_HHMMSS')]);
end
if ~isfolder(opts.OutputDir)
    mkdir(opts.OutputDir);
end

cam = webcam(webcamIdx);
cleanupObj = onCleanup(@() clearCamera(cam)); %#ok<NASGU>
cam.Resolution = sprintf('%dx%d', opts.Resolution(1), opts.Resolution(2));
applyCameraSettings(cam, cfg, 'full');
trySet(cam, 'ExposureMode', 'manual');
trySet(cam, 'WhiteBalanceMode', 'manual');
trySet(cam, 'FocusMode', 'manual');
trySet(cam, 'Focus', opts.Focus);
trySet(cam, 'Sharpness', opts.Sharpness);
trySet(cam, 'Exposure', opts.Exposure);
trySet(cam, 'Gain', opts.Gain);

for k = 1:round(opts.WarmupFrames)
    safeSnapshot(cam, 5);
end
frame = safeSnapshot(cam, 8);
gray = single(rgb2gray(frame)) / 255;
[h, w] = size(gray);

nRows = round(opts.Rows);
nCols = round(opts.Cols);
tileH = floor(h / nRows);
tileW = floor(w / nCols);
lapVar = zeros(nRows, nCols);
tenengrad = zeros(nRows, nCols);
meanGray = zeros(nRows, nCols);

for r = 1:nRows
    for c = 1:nCols
        y1 = (r - 1) * tileH + 1;
        x1 = (c - 1) * tileW + 1;
        if r == nRows, y2 = h; else, y2 = r * tileH; end
        if c == nCols, x2 = w; else, x2 = c * tileW; end
        tile = gray(y1:y2, x1:x2);
        m = metrics(tile);
        lapVar(r,c) = m.lapVar;
        tenengrad(r,c) = m.tenengrad;
        meanGray(r,c) = m.meanGray;
    end
end

result = struct();
result.outputDir = opts.OutputDir;
result.lapVar = lapVar;
result.tenengrad = tenengrad;
result.meanGray = meanGray;
result.frameFile = fullfile(opts.OutputDir, 'frame.png');
result.lapCsv = fullfile(opts.OutputDir, 'lapvar_tiles.csv');
result.tenengradCsv = fullfile(opts.OutputDir, 'tenengrad_tiles.csv');
result.meanCsv = fullfile(opts.OutputDir, 'mean_tiles.csv');
result.lapHeatmap = fullfile(opts.OutputDir, 'lapvar_heatmap.png');
result.tenengradHeatmap = fullfile(opts.OutputDir, 'tenengrad_heatmap.png');

imwrite(frame, result.frameFile);
csvwrite(result.lapCsv, lapVar);
csvwrite(result.tenengradCsv, tenengrad);
csvwrite(result.meanCsv, meanGray);
writeHeatmap(lapVar, result.lapHeatmap, 'Laplacian variance');
writeHeatmap(tenengrad, result.tenengradHeatmap, 'Tenengrad');
save(fullfile(opts.OutputDir, 'spatial_map.mat'), 'result');

fprintf('Spatial map saved: %s\n', opts.OutputDir);
fprintf('Tenengrad min/median/max: %.4g / %.4g / %.4g\n', ...
    min(tenengrad(:)), median(tenengrad(:)), max(tenengrad(:)));

end


function m = metrics(gray)
lapKernel = single([0 1 0; 1 -4 1; 0 1 0]);
sobelX = single([-1 0 1; -2 0 2; -1 0 1]) / 8;
sobelY = sobelX.';
lap = conv2(gray, lapKernel, 'valid');
gx = conv2(gray, sobelX, 'valid');
gy = conv2(gray, sobelY, 'valid');
grad2 = gx.^2 + gy.^2;
m.meanGray = mean(gray(:));
m.lapVar = var(lap(:));
m.tenengrad = mean(grad2(:));
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


function trySet(cam, prop, val)
try
    cam.(prop) = val;
catch
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
