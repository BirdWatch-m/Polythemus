% MAIN  BirdTracker live pipeline.
%
%   Full-pipeline live session: acquires frames from all N cameras, runs
%   per-camera detection, cross-camera association, multi-view triangulation,
%   and 3D Kalman tracking. Logs all data and saves a session file on exit.
%
%   PRE-REQUISITES (both files must exist before running):
%     cfg.skyMaskFile — produced by drawSkyMasks.m at the current resolution
%     cfg.calFile     — produced by calibrateExtrinsics.m
%     Verify cfg.camIndices against webcamlist() each session — the ordering
%     is OS-driven and can change between reboots or replugs.
%
%   Press Q (with the live figure focused) to stop.
%
%   See also: buildConfig, initSystem, recordSession, processRecording

clc; close all; clear;

addpath(genpath(fileparts(mfilename('fullpath'))));

cfg   = buildConfig();
state = initSystem(cfg);    % cameras, buffers, calibration, tracking, log


% Q-to-stop figure. renderFrame finds this figure by name; create it here
% so the key handler is attached before the first render call.
fig = figure('Name', 'BirdTracker — live', 'NumberTitle', 'off', ...
             'KeyPressFcn', @(~,e) setappdata(gcf, 'lastKey', e.Key));
setappdata(fig, 'lastKey', '');

fprintf('Running %d-camera full pipeline at %dx%d. Press Q to stop.\n', ...
        cfg.N, cfg.resolution(1), cfg.resolution(2));

frameCount    = 0;
prevTimestamp = 0;                     % mean frame timestamp from the previous iteration
accAcq = 0; accDet = 0; accPipe = 0; accDisp = 0;

while ishandle(fig)

    % 1. Acquire.
    tA = tic;
    [frames, timestamps] = acquireFrames(state.cams, state.tStart, cfg);
    syncOk = syncCheck(timestamps, cfg);
    accAcq = accAcq + toc(tA) * 1000;

    % 2. Ring buffer + detection (gray conversion done once here; PERF-1).
    tD = tic;
    [state.ringBuf, state.ringIdx, grayFrames] = ...
        updateRingBuf(state.ringBuf, state.ringIdx, frames, cfg);
    [blobs, state] = detectBlobs(grayFrames, state, cfg);
    accDet = accDet + toc(tD) * 1000;

    % 3. Association -> triangulation -> tracking.
    tP = tic;

    groups = associateViews(blobs, state.calibration, cfg);
    points = triangulateGroups(groups, state.calibration, cfg);

    valid = points([points.valid]);
    if isempty(valid)
        meas        = zeros(0, 3);
        meanReprErr = NaN;
    else
        meas        = vertcat(valid.position);
        meanReprErr = mean([valid.reprojErr]);
    end

    % dt: actual inter-frame interval; nominal fps for the first frame.
    frameCount = frameCount + 1;
    if frameCount == 1
        dt = 1 / cfg.fps;
    else
        dt = mean(timestamps) - prevTimestamp;
    end
    prevTimestamp = mean(timestamps);

    [state.tracks, state.nextTrackId] = ...
        updateTracks(state.tracks, state.nextTrackId, meas, dt, cfg);

    accPipe = accPipe + toc(tP) * 1000;

    % 4. Log.
    state.log = logFrame(state.log, blobs, timestamps, syncOk, cfg);
    nConfirmed = sum(strcmp({state.tracks.state}, 'confirmed'));
    state.log.nGroups(end+1)     = numel(groups);
    state.log.nConfirmed(end+1)  = nConfirmed;
    state.log.meanReprErr(end+1) = meanReprErr;
    if nConfirmed > 0
        confirmed = state.tracks(strcmp({state.tracks.state}, 'confirmed'));
        state.log.trackPositions{end+1} = vertcat(confirmed.lastPos);
    else
        state.log.trackPositions{end+1} = zeros(0, 3);
    end

    % 5. Display.
    tR = tic;
    renderFrame(frames, blobs, cfg);
    accDisp = accDisp + toc(tR) * 1000;

    % Console report every 30 frames.
    if mod(frameCount, 30) == 0
        blobCounts = cellfun(@numel, blobs);
        fillPct    = min(100, round(100 * frameCount / cfg.medianBufLen));
        loopMs     = (accAcq + accDet + accPipe + accDisp) / 30;
        fprintf(['t=%5.1fs | frame %4d | buf:%3d%% | blobs/cam: %s | groups: %d | confirmed: %d\n' ...
                 '         ms/frame (avg 30): acq %.1f | det %.1f | 3D %.1f | render %.1f | total %.1f (~%.1f fps)\n'], ...
                toc(state.tStart), frameCount, fillPct, mat2str(blobCounts), ...
                numel(groups), nConfirmed, ...
                accAcq/30, accDet/30, accPipe/30, accDisp/30, loopMs, 1000/loopMs);
        accAcq = 0; accDet = 0; accPipe = 0; accDisp = 0;
    end

    if strcmp(getappdata(fig, 'lastKey'), 'q'), break; end
end

if ishandle(fig), close(fig); end
state.cams = {};
if frameCount > 0
    saveSession(state.log, cfg);
end
fprintf('Stopped after %d frames.\n', frameCount);
