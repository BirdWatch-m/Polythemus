% PROCESSRECORDING  Run the pipeline on a recorded session, offline.
%
%   Script. Set recordingDir + runMode below, then run with F5. Reads a session
%   captured by recordSession and runs detection (and, in 'full' mode, the
%   association -> triangulation -> tracking stages) frame by frame.
%
%   Detection results are valid now and good for offline threshold/morphology
%   tuning. In 'full' mode the full pipeline runs offline on the saved frames.
%
%   Output: results.mat in the recording folder. Always written:
%     results.nBlobs        [N x nFrames]  blob counts per camera
%     results.blobCentroids {1 x nFrames}  {1xN} centroid arrays per frame
%   Additional fields written in 'full' mode:
%     results.nGroups       [1 x nFrames]  cross-camera association groups
%     results.nPoints       [1 x nFrames]  valid triangulated points
%     results.meanReprErr   [1 x nFrames]  mean reprojection error (px); NaN if no points
%     results.nConfirmed    [1 x nFrames]  confirmed 3D tracks
%     results.trackPositions{1 x nFrames}  [nConfirmed x 3] positions per frame
%     results.finalTracks                  track struct array at session end
%
%   See also: recordSession, replaySession, detectBlobs, associateViews, triangulateGroups, updateTracks

clc; close all; clear;
addpath(genpath(fileparts(mfilename('fullpath'))));

% =========================================================================
% USER INPUTS
% =========================================================================
recordingDir = 'output/recordings/20260617_174115';
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
    sessionCalFile = fullfile(recordingDir, 'multiCamParams.mat');
    if isfield(session, 'calibration') && ~isempty(session.calibration)
        multiCamParams = session.calibration;
        fprintf('Using calibration snapshot embedded in session.mat.\n');
    elseif isfile(sessionCalFile)
        calLoaded = load(sessionCalFile);
        if ~isfield(calLoaded, 'multiCamParams')
            error('processRecording:badSessionCalib', ...
                  'Session calibration file is missing multiCamParams: %s', sessionCalFile);
        end
        multiCamParams = calLoaded.multiCamParams;
        fprintf('Using session-local calibration: %s\n', sessionCalFile);
    elseif isfile(cfg.calFile)
        warning('processRecording:globalCalibFallback', ...
                ['No calibration snapshot found in this recording.\n' ...
                 'Falling back to current global calibration: %s'], cfg.calFile);
        calLoaded = load(cfg.calFile);
        if ~isfield(calLoaded, 'multiCamParams')
            error('processRecording:badGlobalCalib', ...
                  'Global calibration file is missing multiCamParams: %s', cfg.calFile);
        end
        multiCamParams = calLoaded.multiCamParams;
    else
        error('processRecording:noCalib', ...
              ['Full mode needs a calibration snapshot in the recording folder or %s.\n' ...
               'Run calibrateExtrinsics first.'], cfg.calFile);
    end
    calibration = buildFundamentalMatrices(multiCamParams, N);
    tracks = struct('id', {}, 'state', {}, 'kf', {}, 'age', {}, 'noDetAge', {}, 'lastPos', {});
    nextId = 1;

    ts  = mean(session.log.timestamps, 1);     % mean timestamp per frame
    dts = [1/cfg.fps, diff(ts)];               % per-frame dt (first = nominal)

end

% --- Results log ---
results.nBlobs        = zeros(N, nFrames);
results.blobCentroids = cell(1, nFrames);
if doFull
    results.nGroups        = zeros(1, nFrames);
    results.nPoints        = zeros(1, nFrames);
    results.meanReprErr    = nan(1, nFrames);
    results.nConfirmed     = zeros(1, nFrames);
    results.trackPositions = cell(1, nFrames);
end

fprintf('Processing %d frames from %s (mode: %s)\n', nFrames, recordingDir, runMode);

for k = 1:nFrames

    % Read this frame from each camera.
    frames = cell(1, N);
    for i = 1:N
        frames{i} = imread(fullfile(recordingDir, sprintf('cam%d', i), sprintf('frame_%06d.tif', k)));
    end

    % Detection (gray-frame interface — updateRingBuf converts once and returns it).
    [state.ringBuf, state.ringIdx, grayFrames] = updateRingBuf(state.ringBuf, state.ringIdx, frames, cfg);
    [blobs, state] = detectBlobs(grayFrames, state, cfg);
    results.nBlobs(:, k)    = cellfun(@numel, blobs).';
    results.blobCentroids{k} = cellfun(@(b) vertcat(b.centroid), blobs, 'UniformOutput', false);

    if doFull
        groups = associateViews(blobs, calibration, cfg);
        points = triangulateGroups(groups, calibration, cfg);
        valid  = points([points.valid]);
        if isempty(valid), meas = zeros(0, 3); else, meas = vertcat(valid.position); end

        [tracks, nextId] = updateTracks(tracks, nextId, meas, dts(k), cfg);

        results.nGroups(k)    = numel(groups);
        results.nPoints(k)    = numel(valid);
        results.meanReprErr(k) = mean([valid.reprojErr]);   % NaN when valid is empty
        results.nConfirmed(k) = sum(strcmp({tracks.state}, 'confirmed'));

        confirmed = tracks(strcmp({tracks.state}, 'confirmed'));
        if isempty(confirmed)
            results.trackPositions{k} = zeros(0, 3);
        else
            results.trackPositions{k} = vertcat(confirmed.lastPos);
        end
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
