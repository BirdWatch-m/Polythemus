% RECORDSESSION  Capture synchronized frames from all cameras to disk.
%
%   Script. Edit the USER INPUTS, then run with F5. Captures only (no detection)
%   to keep the frame rate up; process the result later with processRecording.
%   Press Q in the figure to stop (or set nSeconds).
%
%   Output: output/recordings/<timestamp>/cam<i>/frame_NNNNNN.jpg  per camera,
%   plus session.mat (per-frame timestamps, inter-camera sync, cfg, camera
%   settings) so the recording can be reprocessed and replayed with true timing.
%
%   NOTE: storage is GB-scale (a few minutes of 2x720p JPEG is several GB).
%
%   See also: processRecording, acquireFrames, replaySession

clc; close all; clear;

% =========================================================================
% USER INPUTS
% =========================================================================
nSeconds    = 0;                    % auto-stop after N seconds (0 = Q key only)
jpegQuality = 90;                   % JPEG quality (lower = smaller files)
outRoot     = 'output/recordings';

cfg = buildConfig();
N = cfg.N;
W = cfg.resolution(1);
H = cfg.resolution(2);

% --- Open cameras (logical i -> physical webcamlist index) + warmup ---
cams = cell(1, N);
for i = 1:N
    cams{i} = webcam(cfg.camIndices(i));
    cams{i}.Resolution = sprintf('%dx%d', W, H);
end
warnState = warning('off', 'all');
for i = 1:N
    tWarm = tic;
    while toc(tWarm) < 2.0, try, snapshot(cams{i}); catch, end, end
end
warning(warnState);

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
        fname = fullfile(sessionDir, sprintf('cam%d', i), sprintf('frame_%06d.jpg', frameCount));
        imwrite(frames{i}, fname, 'Quality', jpegQuality);
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
