% REPLAYSESSION  Reconstruct and save annotated video from raw frames + session log.
%
%   Run this script after a recording session to generate an annotated
%   replay video without affecting real-time performance.
%
%   Reads raw frames from frameDir and blob positions from the session log,
%   draws overlays identically to the live display, and writes to a video file.
%
%   USER INPUTS — edit these before running
frameDir   = 'output/frames/';
logFile    = 'output/';          % path to session .mat — leave empty to pick latest
videoOut   = 'output/replay.avi';
playbackFps = 30;         % fps of output video; set lower to slow down replay

% -------------------------------------------------------------------------

cfg   = buildConfig();
cfg.N = 1;   % adjust if multi-camera session

% Load log — use specified file or find the most recent in output/.
if isempty(logFile)
    files = dir(fullfile(cfg.logDir, 'session_*.mat'));
    if isempty(files)
        error('No session files found in %s', cfg.logDir);
    end
    [~, idx] = max([files.datenum]);
    logFile  = fullfile(cfg.logDir, files(idx).name);
end

fprintf('Loading log: %s\n', logFile);
loaded = load(logFile);
log    = loaded.log;

% Find all saved frames, sorted by number.
frameFiles = dir(fullfile(frameDir, 'frame_*.png'));
if isempty(frameFiles)
    error('No frames found in %s', frameDir);
end
[~, order]  = sort({frameFiles.name});
frameFiles  = frameFiles(order);
nFrames     = numel(frameFiles);

fprintf('Found %d frames. Building video...\n', nFrames);

% Initialise video writer.
if ~isfolder(fileparts(videoOut))
    mkdir(fileparts(videoOut));
end
vw          = VideoWriter(videoOut);
vw.FrameRate = playbackFps;
open(vw);

% Render each frame with overlay and write to video.
for k = 1:nFrames

    frame = imread(fullfile(frameDir, frameFiles(k).folder, frameFiles(k).name));

    % Retrieve blob data for this frame from log.
    % log.nBlobs is [N x nFrames]; blob centroids are not stored yet —
    % only counts. For full centroid replay, see note below.
    nBlobs = log.nBlobs(1, min(k, size(log.nBlobs, 2)));
    t      = log.timestamps(min(k, numel(log.timestamps)));
    syncOk = log.syncFlags(min(k, numel(log.syncFlags)));

    % Build annotation text.
    infoStr = sprintf('t=%.2fs | blobs: %d | sync: %s', ...
                      t, nBlobs, mat2str(syncOk));

    % Draw info bar at top of frame.
    display = insertText(frame, [10 10], infoStr, ...
                         'FontSize', 14, 'BoxColor', 'black', ...
                         'TextColor', 'white', 'BoxOpacity', 0.6);

    if k <= numel(log.blobCentroids) && ~isempty(log.blobCentroids{k}{1})
        centroids = log.blobCentroids{k}{1};
        circles   = [centroids, repmat(6, size(centroids,1), 1)];
        display   = insertShape(display, 'Circle', circles, ...
                            'Color', 'green', 'LineWidth', 2);
    end

    writeVideo(vw, display);

    if mod(k, 100) == 0
        fprintf('  Written %d/%d frames\n', k, nFrames);
    end
end

close(vw);
fprintf('Done. Video saved to %s\n', videoOut);
