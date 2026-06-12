% PROCESSRECORDING  Run the pipeline on a recorded session, offline.
%
%   Script. Set recordingDir + runMode below, then run with F5. Reads a session
%   captured by recordSession and runs detection (and, in 'full' mode, the
%   association -> triangulation -> tracking stages) frame by frame.
%
%   Detection results are valid now and good for offline threshold/morphology
%   tuning. In 'full' mode the 3D positions are STRUCTURALLY produced but their
%   absolute values are PENDING BUG-5 calibration validation (see ASSUMPTIONS.md).
%
%   Output: results.mat in the recording folder (per-frame counts; final tracks).
%
%   See also: recordSession, detectBlobs, associateViews, triangulateGroups, updateTracks

clc; close all; clear;

% =========================================================================
% USER INPUTS
% =========================================================================
recordingDir = 'output/recordings/REPLACE_WITH_SESSION_FOLDER';
runMode      = 'detect';    % 'detect' | 'full'

% --- Load the recording ---
sessionFile = fullfile(recordingDir, 'session.mat');
if ~isfile(sessionFile)
    error('processRecording:noSession', 'session.mat not found in %s', recordingDir);
end
loaded  = load(sessionFile);
session = loaded.session;
cfg     = session.cfg;
N       = cfg.N;
H       = cfg.resolution(2);
W       = cfg.resolution(1);
nFrames = session.nFrames;

doFull = strcmp(runMode, 'full');

% --- Per-camera detection state (full-frame masks) ---
state.ringBuf             = cell(1, N);
state.ringIdx             = ones(1, N);
state.bgMedian            = cell(1, N);
state.bgFramesSinceUpdate = zeros(1, N);
state.fgDetectors         = cell(1, N);
state.skyMask             = cell(1, N);
for i = 1:N
    state.ringBuf{i}     = zeros(H, W, cfg.ringBufLen, 'uint8');
    state.bgMedian{i}    = zeros(H, W, 'double');
    state.fgDetectors{i} = vision.ForegroundDetector('NumGaussians', 3, ...
        'NumTrainingFrames', 60, 'LearningRate', 0.005);
    state.skyMask{i}     = true(H, W);
end

% --- Full pipeline: calibration + per-frame dt ---
if doFull
    if ~isfile(cfg.calFile)
        error('processRecording:noCalib', 'Full mode needs %s (run calibrateExtrinsics).', cfg.calFile);
    end
    calLoaded   = load(cfg.calFile);
    calibration = buildFundamentalMatrices(calLoaded.multiCamParams, N);
    tracks = struct('id', {}, 'state', {}, 'kf', {}, 'age', {}, 'noDetAge', {}, 'lastPos', {});
    nextId = 1;

    ts  = mean(session.log.timestamps, 1);     % mean timestamp per frame
    dts = [1/cfg.fps, diff(ts)];               % per-frame dt (first = nominal)
    fprintf('FULL pipeline — 3D positions are PENDING BUG-5 validation.\n');
end

% --- Results log ---
results.nBlobs = zeros(N, nFrames);
if doFull
    results.nGroups    = zeros(1, nFrames);
    results.nPoints    = zeros(1, nFrames);   % valid triangulations
    results.nConfirmed = zeros(1, nFrames);
end

fprintf('Processing %d frames from %s (mode: %s)\n', nFrames, recordingDir, runMode);

for k = 1:nFrames

    % Read this frame from each camera.
    frames = cell(1, N);
    for i = 1:N
        frames{i} = imread(fullfile(recordingDir, sprintf('cam%d', i), sprintf('frame_%06d.jpg', k)));
    end

    % Detection (gray-frame interface — updateRingBuf converts once and returns it).
    [state.ringBuf, state.ringIdx, grayFrames] = updateRingBuf(state.ringBuf, state.ringIdx, frames, cfg);
    [blobs, state] = detectBlobs(grayFrames, state, cfg);
    results.nBlobs(:, k) = cellfun(@numel, blobs).';

    if doFull
        groups = associateViews(blobs, calibration, cfg);
        points = triangulateGroups(groups, calibration, cfg);
        valid  = points([points.valid]);
        if isempty(valid), meas = zeros(0, 3); else, meas = vertcat(valid.position); end

        [tracks, nextId] = updateTracks(tracks, nextId, meas, dts(k), cfg);

        results.nGroups(k)    = numel(groups);
        results.nPoints(k)    = numel(valid);
        results.nConfirmed(k) = sum(strcmp({tracks.state}, 'confirmed'));
    end

    if mod(k, 30) == 0
        if doFull
            fprintf('frame %4d/%d | blobs/cam %s | groups %d | pts %d | confirmed %d\n', ...
                    k, nFrames, mat2str(results.nBlobs(:,k).'), results.nGroups(k), ...
                    results.nPoints(k), results.nConfirmed(k));
        else
            fprintf('frame %4d/%d | blobs/cam %s\n', k, nFrames, mat2str(results.nBlobs(:,k).'));
        end
    end
end

if doFull, results.finalTracks = tracks; end
save(fullfile(recordingDir, 'results.mat'), 'results');
fprintf('Done. Results saved to %s\n', fullfile(recordingDir, 'results.mat'));
