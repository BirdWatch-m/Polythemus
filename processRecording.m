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
%     results.frameIndex    [1 x nFrames]  absolute frame numbers in session
%     results.nGroups       [1 x nFrames]  cross-camera association groups
%     results.nPoints       [1 x nFrames]  valid triangulated points
%     results.meanReprErr   [1 x nFrames]  mean reprojection error (px); NaN if no points
%     results.nConfirmed    [1 x nFrames]  confirmed 3D tracks
%     results.trackIds{1 x nFrames}        [nConfirmed x 1] stable track IDs
%     results.trackPositions{1 x nFrames}  [nConfirmed x 3] positions per frame
%     results.finalTracks                  track struct array at session end
%
%   See also: recordSession, replaySession, detectBlobs, associateViews, triangulateGroups, updateTracks

clc; close all; clear;
addpath(genpath(fileparts(mfilename('fullpath'))));

% =========================================================================
% USER INPUTS
% =========================================================================
recordingDir = 'output/recordings/20260621_134509';
runMode      = 'full';    % 'detect' | 'full'

% --- Load the recording ---
sessionFile = fullfile(recordingDir, 'session.mat');
if ~isfile(sessionFile)
    error('processRecording:noSession', 'session.mat not found in %s', recordingDir);
end
loaded  = load(sessionFile);
session = loaded.session;
cfg     = buildConfig();               % always use current tuning
cfg.N          = session.cfg.N;        % inherit hardware params from recording
cfg.resolution = session.cfg.resolution;
cfg.fps        = session.cfg.fps;
cfg.ringBufLen = session.cfg.ringBufLen;
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
results.frameIndex    = 1:nFrames;
if doFull
    results.nGroups        = zeros(1, nFrames);
    results.nPoints        = zeros(1, nFrames);
    results.meanReprErr    = nan(1, nFrames);
    results.nConfirmed     = zeros(1, nFrames);
    results.trackIds       = cell(1, nFrames);
    results.trackPositions = cell(1, nFrames);
end

% --- Gate counter accumulators ---
if doFull
    stats.det  = struct('rawRegions', 0, 'rejSmall', 0, 'rejLarge', 0, 'rejAspect', 0, 'passed', 0);
    stats.assoc = struct('candidatePairs', 0, 'rejectedEpi', 0, 'matchedPairs', 0, ...
                         'singletonGroups', 0, 'multiViewGroups', 0);
    stats.tri  = struct('skippedFewViews', 0, 'failedRepr', 0, 'failedNegZ', 0, ...
                        'failedMaxRange', 0, 'valid', 0);
    stats.trk  = struct('measMatched', 0, 'newSpawned', 0, 'tentativeDropped', 0, 'coastedOut', 0);
    stats.out  = struct('confirmed', 0, 'suppressed', 0, 'reported', 0);
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
    [blobs, state, dcDet] = detectBlobs(grayFrames, state, cfg);
    results.nBlobs(:, k)    = cellfun(@numel, blobs).';
    results.blobCentroids{k} = cellfun(@(b) vertcat(b.centroid), blobs, 'UniformOutput', false);

    if doFull
        [groups, dcAssoc] = associateViews(blobs, calibration, cfg);
        [points, dcTri]   = triangulateGroups(groups, calibration, cfg);
        valid  = points([points.valid]);
        if isempty(valid), meas = zeros(0, 3); else, meas = vertcat(valid.position); end

        [tracks, nextId, dcTrk] = updateTracks(tracks, nextId, meas, dts(k), cfg);

        results.nGroups(k)     = numel(groups);
        results.nPoints(k)     = numel(valid);
        results.meanReprErr(k) = mean([valid.reprojErr]);   % NaN when valid is empty
        results.nConfirmed(k)  = sum(strcmp({tracks.state}, 'confirmed') & [tracks.age] >= cfg.minTrackAge);

        confirmed = tracks(strcmp({tracks.state}, 'confirmed') & [tracks.age] >= cfg.minTrackAge);
        if isempty(confirmed)
            results.trackIds{k}       = zeros(0, 1);
            results.trackPositions{k} = zeros(0, 3);
        else
            results.trackIds{k}       = vertcat(confirmed.id);
            results.trackPositions{k} = vertcat(confirmed.lastPos);
        end

        % Accumulate gate counters.
        stats.det.rawRegions  = stats.det.rawRegions  + dcDet.rawRegions;
        stats.det.rejSmall    = stats.det.rejSmall    + dcDet.rejSmall;
        stats.det.rejLarge    = stats.det.rejLarge    + dcDet.rejLarge;
        stats.det.rejAspect   = stats.det.rejAspect   + dcDet.rejAspect;
        stats.det.passed      = stats.det.passed      + dcDet.passed;

        stats.assoc.candidatePairs  = stats.assoc.candidatePairs  + dcAssoc.candidatePairs;
        stats.assoc.rejectedEpi     = stats.assoc.rejectedEpi     + dcAssoc.rejectedEpi;
        stats.assoc.matchedPairs    = stats.assoc.matchedPairs    + dcAssoc.matchedPairs;
        stats.assoc.singletonGroups = stats.assoc.singletonGroups + dcAssoc.singletonGroups;
        stats.assoc.multiViewGroups = stats.assoc.multiViewGroups + dcAssoc.multiViewGroups;

        stats.tri.skippedFewViews = stats.tri.skippedFewViews + dcTri.skippedFewViews;
        stats.tri.failedRepr      = stats.tri.failedRepr      + dcTri.failedRepr;
        stats.tri.failedNegZ      = stats.tri.failedNegZ      + dcTri.failedNegZ;
        stats.tri.failedMaxRange  = stats.tri.failedMaxRange  + dcTri.failedMaxRange;
        stats.tri.valid           = stats.tri.valid           + dcTri.valid;

        stats.trk.measMatched      = stats.trk.measMatched      + dcTrk.measMatched;
        stats.trk.newSpawned       = stats.trk.newSpawned       + dcTrk.newSpawned;
        stats.trk.tentativeDropped = stats.trk.tentativeDropped + dcTrk.tentativeDropped;
        stats.trk.coastedOut       = stats.trk.coastedOut       + dcTrk.coastedOut;

        nAllConf = sum(strcmp({tracks.state}, 'confirmed'));
        stats.out.confirmed  = stats.out.confirmed  + nAllConf;
        stats.out.suppressed = stats.out.suppressed + (nAllConf - results.nConfirmed(k));
        stats.out.reported   = stats.out.reported   + results.nConfirmed(k);
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

% --- End-of-run gate report ---
if doFull
    printGateReport(stats, nFrames, N, cfg);
end


% =========================================================================
% LOCAL HELPERS
% =========================================================================

function printGateReport(s, nFrames, N, cfg)
fr  = nFrames;
sep = repmat('=', 1, 62);
fprintf('\n%s\n', sep);
fprintf('Pipeline gate report — %d frames, %d cameras\n', fr, N);
fprintf('%s\n\n', sep);

fprintf('Detection (summed across %d cameras):\n', N);
fprintf('  Raw regions          %8d  (%6.1f /fr/cam)\n', s.det.rawRegions, s.det.rawRegions/fr/N);
fprintf('  Rejected too small   %8d  (%6.1f /fr/cam)\n', s.det.rejSmall,   s.det.rejSmall/fr/N);
fprintf('  Rejected too large   %8d  (%6.1f /fr/cam)\n', s.det.rejLarge,   s.det.rejLarge/fr/N);
fprintf('  Rejected aspect      %8d  (%6.1f /fr/cam)\n', s.det.rejAspect,  s.det.rejAspect/fr/N);
fprintf('  Blobs passed         %8d  (%6.1f /fr/cam)\n', s.det.passed,     s.det.passed/fr/N);

fprintf('\nAssociation:\n');
if s.assoc.candidatePairs > 0
    epiPct = 100 * s.assoc.rejectedEpi / s.assoc.candidatePairs;
    epiStr = sprintf('%.1f%% of candidates', epiPct);
else
    epiStr = 'no candidates';
end
fprintf('  Candidate pairs      %8d  (%6.1f /fr)\n',    s.assoc.candidatePairs,  s.assoc.candidatePairs/fr);
fprintf('  Rejected epi gate    %8d  (%6.1f /fr)  [%s]\n', s.assoc.rejectedEpi, s.assoc.rejectedEpi/fr, epiStr);
fprintf('  Matched pairs        %8d  (%6.1f /fr)\n',    s.assoc.matchedPairs,    s.assoc.matchedPairs/fr);
fprintf('  Singleton groups     %8d  (%6.1f /fr)\n',    s.assoc.singletonGroups, s.assoc.singletonGroups/fr);
fprintf('  Multi-view groups    %8d  (%6.1f /fr)\n',    s.assoc.multiViewGroups, s.assoc.multiViewGroups/fr);

fprintf('\nTriangulation:\n');
fprintf('  Skipped (< 2 views)  %8d  (%6.1f /fr)\n', s.tri.skippedFewViews, s.tri.skippedFewViews/fr);
fprintf('  Failed reprErr       %8d  (%6.1f /fr)\n', s.tri.failedRepr,      s.tri.failedRepr/fr);
fprintf('  Failed negative-Z    %8d  (%6.1f /fr)\n', s.tri.failedNegZ,      s.tri.failedNegZ/fr);
fprintf('  Failed maxRange      %8d  (%6.1f /fr)\n', s.tri.failedMaxRange,  s.tri.failedMaxRange/fr);
fprintf('  Valid points         %8d  (%6.1f /fr)\n', s.tri.valid,           s.tri.valid/fr);

fprintf('\nTracking:\n');
fprintf('  Matched to track     %8d  (%6.1f /fr)\n', s.trk.measMatched,      s.trk.measMatched/fr);
fprintf('  New tracks spawned   %8d  (%6.1f /fr)\n', s.trk.newSpawned,       s.trk.newSpawned/fr);
fprintf('  Tentative drops      %8d  (%6.1f /fr)\n', s.trk.tentativeDropped, s.trk.tentativeDropped/fr);
fprintf('  Coasted out          %8d  (%6.1f /fr)\n', s.trk.coastedOut,       s.trk.coastedOut/fr);

fprintf('\nOutput (minTrackAge = %d):\n', cfg.minTrackAge);
fprintf('  Confirmed track-frames   %8d\n', s.out.confirmed);
fprintf('  Suppressed by age        %8d\n', s.out.suppressed);
fprintf('  Reported in results      %8d\n', s.out.reported);
fprintf('%s\n', sep);
end
