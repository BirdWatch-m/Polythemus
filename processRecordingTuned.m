% PROCESSRECORDINGTUNED  processRecording with locally-overridden detection params.
%
%   One-off diagnostic script. Processes a fixed frame window of one recording
%   with tuned parameters (medianFgThreshold=20, morphKernelRadius=1) WITHOUT
%   touching buildConfig. Saves to results_tuned.mat so the original
%   results.mat is never overwritten.
%
%   See also: processRecording, replaySessionTuned

clc; close all; clear;
addpath(genpath(fileparts(mfilename('fullpath'))));

% =========================================================================
% USER INPUTS
% =========================================================================
recordingDir = 'output/recordings/20260620_162311';
frameStart   = 5143;    % first frame of the window (90 frames before bird at t=200s)
frameEnd     = 5961;    % last  frame of the window (90*5 frames after bird at t=210s)
warmupFrames = 90;      % ring-buffer + GMM warmup before frameStart
runMode      = 'full';  % 'detect' | 'full'

% =========================================================================
% PARAMETER OVERRIDES  (local only — buildConfig is NOT changed)
% =========================================================================
loaded  = load(fullfile(recordingDir, 'session.mat'));
session = loaded.session;
cfg     = session.cfg;          % start from the recorded config

cfg.minBlobArea   = 4;      % was 9  — pigeon is 4px² at thr=35; morphology already handles speckle
cfg.useMorphology = false;  % was true — disk-1 erosion kills a 4px cluster before it reaches gating

fprintf('Parameter overrides applied:\n');
fprintf('  minBlobArea   : %g  (was %g)\n', cfg.minBlobArea,   session.cfg.minBlobArea);
fprintf('  useMorphology : %d  (was %d)\n', cfg.useMorphology, session.cfg.useMorphology);

% =========================================================================
% INIT STATE
% =========================================================================
N  = cfg.N;
H  = cfg.resolution(2);
W  = cfg.resolution(1);
doFull = strcmp(runMode, 'full');

state.ringBuf             = cell(1, N);
state.ringIdx             = ones(1, N);
state.bgMedian            = cell(1, N);
state.bgFramesSinceUpdate = zeros(1, N);
state.fgDetectors         = cell(1, N);
state.skyMask             = cell(1, N);
for i = 1:N
    state.ringBuf{i}     = zeros(H, W, cfg.ringBufLen, 'uint8');
    state.bgMedian{i}    = zeros(H, W, 'int16');
    state.fgDetectors{i} = vision.ForegroundDetector('NumGaussians', 3, ...
        'NumTrainingFrames', 60, 'LearningRate', 0.005);
    state.skyMask{i}     = true(H, W);   % offline: no sky mask restriction
end

if doFull
    sessionCalFile = fullfile(recordingDir, 'multiCamParams.mat');
    if isfield(session, 'calibration') && ~isempty(session.calibration)
        multiCamParams = session.calibration;
    elseif isfile(sessionCalFile)
        calLoaded      = load(sessionCalFile);
        multiCamParams = calLoaded.multiCamParams;
    else
        error('processRecordingTuned:noCalib', 'No calibration found in %s', recordingDir);
    end
    calibration = buildFundamentalMatrices(multiCamParams, N);
    tracks  = struct('id', {}, 'state', {}, 'kf', {}, 'age', {}, 'noDetAge', {}, 'lastPos', {});
    nextId  = 1;
    ts      = mean(session.log.timestamps, 1);
    dts     = [1/cfg.fps, diff(ts)];
end

% =========================================================================
% WARMUP  (frames before frameStart — background only, no results logged)
% =========================================================================
warmupStart = max(1, frameStart - warmupFrames);
fprintf('Warming up on frames %d – %d ...\n', warmupStart, frameStart - 1);

for k = warmupStart : frameStart - 1
    frames = cell(1, N);
    for i = 1:N
        frames{i} = imread(fullfile(recordingDir, sprintf('cam%d', i), sprintf('frame_%06d.tif', k)));
    end
    [state.ringBuf, state.ringIdx, grayFrames] = updateRingBuf(state.ringBuf, state.ringIdx, frames, cfg);
    for i = 1:N
        state.bgFramesSinceUpdate(i) = state.bgFramesSinceUpdate(i) + 1;
        nF = min(cfg.medianBufLen, cfg.ringBufLen);
        if state.bgFramesSinceUpdate(i) >= cfg.bgUpdateInterval || all(state.bgMedian{i}(:) == 0)
            state.bgMedian{i} = int16(median(state.ringBuf{i}(:,:,1:cfg.bgMedianStride:nF), 3));
            state.bgFramesSinceUpdate(i) = 0;
        end
        step(state.fgDetectors{i}, grayFrames{i});
    end
end

% =========================================================================
% PROCESS WINDOW
% =========================================================================
nWindow = frameEnd - frameStart + 1;
results.nBlobs        = zeros(N, nWindow);
results.blobCentroids = cell(1, nWindow);
results.frameIndex    = frameStart : frameEnd;   % absolute frame numbers

if doFull
    results.nGroups        = zeros(1, nWindow);
    results.nPoints        = zeros(1, nWindow);
    results.meanReprErr    = nan(1, nWindow);
    results.nConfirmed     = zeros(1, nWindow);
    results.trackIds       = cell(1, nWindow);
    results.trackPositions = cell(1, nWindow);
end

fprintf('Processing frames %d – %d  (%d frames, mode: %s) ...\n', ...
        frameStart, frameEnd, nWindow, runMode);

for ki = 1:nWindow
    k = frameStart + ki - 1;

    frames = cell(1, N);
    for i = 1:N
        frames{i} = imread(fullfile(recordingDir, sprintf('cam%d', i), sprintf('frame_%06d.tif', k)));
    end

    [state.ringBuf, state.ringIdx, grayFrames] = updateRingBuf(state.ringBuf, state.ringIdx, frames, cfg);
    [blobs, state] = detectBlobs(grayFrames, state, cfg);

    results.nBlobs(:, ki)    = cellfun(@numel, blobs).';
    results.blobCentroids{ki} = cellfun(@(b) vertcat(b.centroid), blobs, 'UniformOutput', false);

    if doFull
        groups = associateViews(blobs, calibration, cfg);
        points = triangulateGroups(groups, calibration, cfg);
        valid  = points([points.valid]);
        if isempty(valid), meas = zeros(0, 3); else, meas = vertcat(valid.position); end

        [tracks, nextId] = updateTracks(tracks, nextId, meas, dts(k), cfg);

        results.nGroups(ki)     = numel(groups);
        results.nPoints(ki)     = numel(valid);
        results.meanReprErr(ki) = mean([valid.reprojErr]);
        results.nConfirmed(ki)  = sum(strcmp({tracks.state}, 'confirmed'));

        confirmed = tracks(strcmp({tracks.state}, 'confirmed'));
        if isempty(confirmed)
            results.trackIds{ki}       = zeros(0, 1);
            results.trackPositions{ki} = zeros(0, 3);
        else
            results.trackIds{ki}       = vertcat(confirmed.id);
            results.trackPositions{ki} = vertcat(confirmed.lastPos);
        end
    end

    if mod(ki, 30) == 0
        if doFull
            fprintf('  frame %4d/%d (abs %d) | blobs/cam %s | groups %d | pts %d | confirmed %d\n', ...
                    ki, nWindow, k, mat2str(results.nBlobs(:,ki).'), ...
                    results.nGroups(ki), results.nPoints(ki), results.nConfirmed(ki));
        else
            fprintf('  frame %4d/%d (abs %d) | blobs/cam %s\n', ...
                    ki, nWindow, k, mat2str(results.nBlobs(:,ki).'));
        end
    end
end

if doFull, results.finalTracks = tracks; end

outFile = fullfile(recordingDir, 'results_tuned.mat');
save(outFile, 'results');
fprintf('\n=== Summary ===\n');
fprintf('Frames processed : %d  (abs %d – %d)\n', nWindow, frameStart, frameEnd);
fprintf('Frames w/ blobs cam1 : %d  (%.1f%%)\n', sum(results.nBlobs(1,:)>=1), 100*mean(results.nBlobs(1,:)>=1));
fprintf('Frames w/ blobs cam2 : %d  (%.1f%%)\n', sum(results.nBlobs(2,:)>=1), 100*mean(results.nBlobs(2,:)>=1));
if doFull
    fprintf('Frames w/ confirmed track: %d\n', sum(results.nConfirmed>=1));
    fprintf('Max confirmed tracks/frame: %d\n', max(results.nConfirmed));
    validErr = results.meanReprErr(~isnan(results.meanReprErr));
    if ~isempty(validErr)
        fprintf('Mean reproj error: %.2f px\n', mean(validErr));
    end
end
fprintf('Saved to: %s\n', outFile);
