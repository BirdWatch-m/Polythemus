% RECORDSESSION  Capture synchronized frames from all cameras to disk.
%
%   Script. Edit the USER INPUTS, then run with F5. Captures only (no detection)
%   to keep the frame rate up; process the result later with processRecording.
%   Press Q in the figure to stop (or set nSeconds).
%
%   Output: output/recordings/<timestamp>/cam<i>/frame_NNNNNN.tif  per camera,
%   plus session.mat (per-frame timestamps, inter-camera sync, cfg, camera
%   settings) so the recording can be reprocessed and replayed with true timing.
%
%   NOTE: storage is GB-scale (uncompressed TIFF at 1280x720 is ~2.8 MB/frame/camera).
%   Frames are saved losslessly so no artefacts contaminate background
%   subtraction or blob detection. Requires an SSD for sustained 30 fps.
%
%   See also: processRecording, acquireFrames, replaySession

clc; close all; clear;

% =========================================================================
% USER INPUTS
% =========================================================================
nSeconds = 0;           % auto-stop after N seconds (0 = Q key only)
outRoot  = 'output/recordings';

cfg = buildConfig();
N = cfg.N;
W = cfg.resolution(1);
H = cfg.resolution(2);

% --- Open cameras (logical i -> physical webcamlist index) ---
% Structural settings (focus + per-model profile + auto mode) are applied
% first; all cameras then settle simultaneously so both lock at the same
% scene brightness (sequential per-camera settling can lock at different
% exposures if the scene changes between them).
cams = cell(1, N);
for i = 1:N
    cams{i} = webcam(cfg.camIndices(i));
    cams{i}.Resolution = sprintf('%dx%d', W, H);
    applyCameraSettings(cams{i}, cfg, 'structural');
end

fprintf('Settling all cameras simultaneously (%.0fs)...\n', cfg.autoSettleSeconds);
warnState = warning('off', 'all');
t0 = tic;
while toc(t0) < cfg.autoSettleSeconds
    for i = 1:N
        try, snapshot(cams{i}); catch, end
    end
end
warning(warnState);
for i = 1:N
    settled = applyCameraSettings(cams{i}, cfg, 'lock');
    fprintf('  Cam %d locked: Exposure = %g\n', i, settled.Exposure);
end

% Capture camera settings for reproducibility.
camSettings = cell(1, N);
for i = 1:N
    try, camSettings{i} = get(cams{i}); catch, camSettings{i} = struct('Resolution', cams{i}.Resolution); end
end

% --- Output directories ---
sessionDir = fullfile(outRoot, datestr(now, 'yyyymmdd_HHMMSS'));
for i = 1:N
    d = fullfile(sessionDir, sprintf('cam%d', i));
    if ~isfolder(d), mkdir(d); end
end

% --- Log (preallocated; auto-grows if exceeded, trimmed at end) ---
estFrames = 10000;
log.timestamps = nan(N, estFrames);   % seconds since start, per camera
log.syncMs     = nan(1, estFrames);   % inter-camera spread per frame (ms)

% --- Q-to-stop figure ---
fig = figure('Name', 'Recording — press Q to stop', 'NumberTitle', 'off', ...
             'KeyPressFcn', @(~,e) setappdata(gcf, 'lastKey', e.Key));
setappdata(fig, 'lastKey', '');

fprintf('Recording %d camera(s) at %dx%d to:\n  %s\nPress Q to stop.\n', N, W, H, sessionDir);
tStart = tic;
frameCount = 0;

while ishandle(fig)

    [frames, timestamps] = acquireFrames(cams, tStart, cfg);
    frameCount = frameCount + 1;

    for i = 1:N
        fname = fullfile(sessionDir, sprintf('cam%d', i), sprintf('frame_%06d.tif', frameCount));
        imwrite(rgb2gray(frames{i}), fname, 'Compression', 'none');
    end

    log.timestamps(:, frameCount) = timestamps(:);
    log.syncMs(frameCount)        = (max(timestamps) - min(timestamps)) * 1000;

    if mod(frameCount, 30) == 0
        fprintf('  %d frames | t=%.1fs | sync ~%.1fms | ~%.1f fps\n', ...
                frameCount, toc(tStart), log.syncMs(frameCount), frameCount / toc(tStart));
    end

    if nSeconds > 0 && toc(tStart) >= nSeconds, break; end
    if strcmp(getappdata(fig, 'lastKey'), 'q'), break; end
end

if ishandle(fig), close(fig); end
cams = {};

% Trim and save session metadata.
log.timestamps = log.timestamps(:, 1:frameCount);
log.syncMs     = log.syncMs(1:frameCount);

session.cfg         = cfg;
session.camSettings = camSettings;
session.nFrames     = frameCount;
session.log         = log;
save(fullfile(sessionDir, 'session.mat'), 'session');

fprintf('Done. %d frames/camera saved to %s\n', frameCount, sessionDir);
