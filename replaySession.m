% REPLAYSESSION Writes annotated replay videos from processed results.

clc; close all; clear;

addpath(genpath(fileparts(mfilename('fullpath'))));

recordingDir = 'output/recordings/box';
playbackFps  = 30;
frameRange   = [];

blobMarkerRadius  = 7;
trackMarkerRadius = 12;
trackSnapGatePx   = 14;

sessionFile = fullfile(recordingDir, 'session.mat');
if ~isfile(sessionFile)
    error('replaySession:noSession', 'session.mat not found in %s', recordingDir);
end
loaded  = load(sessionFile);
session = loaded.session;
cfg     = session.cfg;
N       = cfg.N;
nFrames = session.nFrames;

resultsFile = fullfile(recordingDir, 'results.mat');
hasResults  = isfile(resultsFile);
if hasResults
    res = load(resultsFile);
    res = res.results;
    hasCentroids   = isfield(res, 'blobCentroids');
    hasFullResults = isfield(res, 'nConfirmed') && isfield(res, 'trackPositions');
    fprintf('Loaded results.mat (detect-only: %d | full pipeline: %d).\n', ...
            hasResults, hasFullResults);
else
    res = struct();
    hasCentroids   = false;
    hasFullResults = false;
    fprintf('No results.mat found - run processRecording first for detection/track overlay.\n');
end

[frameIndices, resultRows] = resolveReplayWindow(res, nFrames, hasResults, frameRange);
nReplay = numel(frameIndices);
if nReplay == 0
    error('replaySession:emptyWindow', 'No frames match frameRange.');
end

trackOverlay = false;
trackStyles  = struct('ids', [], 'colors', zeros(0, 3));
if hasFullResults
    res = ensureTrackIds(res, resultsFile);
    try
        calibration = loadRecordingCalibration(recordingDir, session, cfg);
        trackStyles = buildTrackStyles(res.trackIds(resultRows));
        trackOverlay = true;
    catch ME
        warning('replaySession:trackOverlayDisabled', ...
                'Track projection overlay disabled: %s', ME.message);
    end
end

for i = 1:N

    videoOut = replayVideoName(recordingDir, i, frameRange);
    vw = VideoWriter(videoOut, 'Motion JPEG AVI');
    vw.FrameRate = playbackFps;
    if isprop(vw, 'Quality'), vw.Quality = 95; end
    open(vw);

    fprintf('Writing cam %d / %d  (%d frames) -> %s\n', i, N, nReplay, videoOut);

    for r = 1:nReplay
        k  = frameIndices(r);
        ri = resultRows(r);

        frame = imread(fullfile(recordingDir, sprintf('cam%d', i), ...
                                sprintf('frame_%06d.tif', k)));

        t      = cameraFrameTime(session, i, k);
        syncMs = frameSyncMs(session, k);

        if hasFullResults
            infoStr = sprintf('cam%d | f%d | t=%.2fs | sync %.1fms | blobs:%d | confirmed:%d | reprErr:%.1fpx', ...
                              i, k, t, syncMs, res.nBlobs(i,ri), res.nConfirmed(ri), res.meanReprErr(ri));
        elseif hasResults
            infoStr = sprintf('cam%d | f%d | t=%.2fs | sync %.1fms | blobs:%d', ...
                              i, k, t, syncMs, res.nBlobs(i, ri));
        else
            infoStr = sprintf('cam%d | f%d | t=%.2fs | sync %.1fms', i, k, t, syncMs);
        end

        annotated = repmat(frame, [1 1 3]);

        annotated = insertText(annotated, [10 10], infoStr, ...
                               'FontSize', 14, 'BoxColor', 'black', ...
                               'TextColor', 'white', 'BoxOpacity', 0.6);

        if hasCentroids
            centroids = res.blobCentroids{ri}{i};
        else
            centroids = zeros(0, 2);
        end
        annotated = drawBlobOverlay(annotated, centroids, blobMarkerRadius);

        if trackOverlay
            annotated = drawTrackOverlay(annotated, res.trackIds{ri}, res.trackPositions{ri}, ...
                                         centroids, calibration, i, trackStyles, ...
                                         trackMarkerRadius, trackSnapGatePx);
        end

        writeVideo(vw, annotated);

        if mod(r, 100) == 0
            fprintf('  cam %d: %d / %d frames\n', i, r, nReplay);
        end
    end

    close(vw);
    fprintf('  Saved: %s\n', videoOut);
end

fprintf('Done.\n');

function [frameIndices, resultRows] = resolveReplayWindow(res, nSessionFrames, hasResults, frameRange)
if hasResults && isfield(res, 'frameIndex')
    allFrameIndices = res.frameIndex(:).';
elseif hasResults && isfield(res, 'nBlobs')
    allFrameIndices = 1:size(res.nBlobs, 2);
else
    allFrameIndices = 1:nSessionFrames;
end
if isempty(frameRange)
    keep = true(size(allFrameIndices));
else
    if numel(frameRange) ~= 2 || frameRange(1) > frameRange(2)
        error('replaySession:badFrameRange', 'frameRange must be [] or [firstFrame lastFrame].');
    end
    keep = allFrameIndices >= frameRange(1) & allFrameIndices <= frameRange(2);
end
frameIndices = allFrameIndices(keep);
resultRows   = find(keep);
end

function name = replayVideoName(recordingDir, camIdx, frameRange)
if isempty(frameRange)
    name = fullfile(recordingDir, sprintf('cam%d_replay.avi', camIdx));
else
    name = fullfile(recordingDir, sprintf('cam%d_replay_%06d_%06d.avi', ...
                    camIdx, frameRange(1), frameRange(2)));
end
end

function annotated = drawBlobOverlay(annotated, centroids, radius)
if isempty(centroids), return; end
circles = [centroids, repmat(radius, size(centroids, 1), 1)];
annotated = insertShape(annotated, 'Circle', circles, ...
                        'Color', 'red', 'LineWidth', 1);
end

function annotated = drawTrackOverlay(annotated, ids, positions, centroids, calibration, camIdx, styles, radius, snapGatePx)
if isempty(ids) || isempty(positions), return; end
ids = ids(:);
if size(positions, 1) ~= numel(ids), return; end
projected = projectWorldToCamera(positions, calibration, camIdx);
[height, width, ~] = size(annotated);
for j = 1:numel(ids)
    p = projected(j, :);
    if ~all(isfinite(p)), continue; end
    markerCenter = snapToNearestBlob(p, centroids, snapGatePx);
    if ~pointInFrame(markerCenter, width, height), continue; end
    color = trackColor(styles, ids(j));
    annotated = insertShape(annotated, 'Circle', [markerCenter radius], ...
                            'Color', color, 'LineWidth', 3);
    labelPos = trackLabelPosition(markerCenter, radius, width, height);
    annotated = insertText(annotated, labelPos, sprintf('ID %d', ids(j)), ...
                           'FontSize', 12, 'BoxColor', color, ...
                           'TextColor', 'white', 'BoxOpacity', 0.65);
end
end

function projected = projectWorldToCamera(worldPoints, calibration, camIdx)
if isempty(worldPoints)
    projected = zeros(0, 2);
    return;
end
K = calibration.intrinsics{camIdx}.IntrinsicMatrix.';
R = calibration.R{camIdx};
t = calibration.t{camIdx}(:);
Xc = R * worldPoints.' + t;
projected = nan(size(worldPoints, 1), 2);
valid = Xc(3, :) > eps;
if any(valid)
    x = K * Xc(:, valid);
    projected(valid, :) = [x(1,:).' ./ x(3,:).', x(2,:).' ./ x(3,:).'];
end
end

function center = snapToNearestBlob(projected, centroids, gatePx)
center = projected;
if isempty(centroids), return; end
d = hypot(centroids(:,1) - projected(1), centroids(:,2) - projected(2));
[minD, idx] = min(d);
if minD <= gatePx
    center = centroids(idx, :);
end
end

function tf = pointInFrame(p, width, height)
tf = p(1) >= 1 && p(1) <= width && p(2) >= 1 && p(2) <= height;
end

function pos = trackLabelPosition(center, radius, width, height)
pos = [center(1) + radius + 5, center(2) - radius - 18];
pos(1) = min(max(pos(1), 1), max(1, width - 70));
pos(2) = min(max(pos(2), 1), max(1, height - 24));
end

function styles = buildTrackStyles(trackIds)
allIds = [];
for k = 1:numel(trackIds)
    allIds = [allIds; trackIds{k}(:)];
end
allIds = unique(allIds(:).');
allIds = allIds(~isnan(allIds));
styles.ids = allIds;
styles.colors = uint8(round(255 * trackPalette(numel(allIds))));
end

function color = trackColor(styles, id)
idx = find(styles.ids == id, 1);
if isempty(idx)
    fallback = uint8(round(255 * trackPalette(max(1, id))));
    color = double(fallback(end, :));
else
    color = double(styles.colors(idx, :));
end
end

function colors = trackPalette(n)
base = [0.000 0.447 0.741;
        0.850 0.325 0.098;
        0.466 0.674 0.188;
        0.494 0.184 0.556;
        0.929 0.694 0.125;
        0.301 0.745 0.933;
        0.635 0.078 0.184;
        0.250 0.250 0.250];
if n <= size(base, 1)
    colors = base(1:n, :);
else
    colors = lines(n);
end
end

function res = ensureTrackIds(res, resultsFile)
if isfield(res, 'trackIds')
    return;
end
n = numel(res.trackPositions);
res.trackIds = cell(1, n);
maxRows = 0;
for k = 1:n
    maxRows = max(maxRows, size(res.trackPositions{k}, 1));
end
for k = 1:n
    nRows = size(res.trackPositions{k}, 1);
    res.trackIds{k} = (1:nRows).';
end
if maxRows > 1
    warning('replaySession:legacyTrackIds', ...
            ['%s has no results.trackIds. Using per-frame row numbers; ' ...
             're-run processRecording for stable track colours and labels.'], resultsFile);
else
    warning('replaySession:legacyTrackIds', ...
            '%s has no results.trackIds. Using single-track fallback ID 1.', resultsFile);
end
end

function calibration = loadRecordingCalibration(recordingDir, session, cfg)
sessionCalFile = fullfile(recordingDir, 'multiCamParams.mat');
if isfield(session, 'calibration') && ~isempty(session.calibration)
    calibration = session.calibration;
elseif isfile(sessionCalFile)
    loaded = load(sessionCalFile);
    calibration = loaded.multiCamParams;
elseif isfield(cfg, 'calFile') && isfile(cfg.calFile)
    loaded = load(cfg.calFile);
    calibration = loaded.multiCamParams;
else
    error('No calibration snapshot found in %s or cfg.calFile.', recordingDir);
end
end

function t = cameraFrameTime(session, camIdx, frameIdx)
t = NaN;
if ~isfield(session, 'log'), return; end
if isfield(session.log, 'cameraTimestamps') && ...
        size(session.log.cameraTimestamps, 1) >= camIdx && ...
        size(session.log.cameraTimestamps, 2) >= frameIdx
    t = session.log.cameraTimestamps(camIdx, frameIdx);
elseif isfield(session.log, 'timestamps')
    ts = session.log.timestamps;
    if ismatrix(ts) && size(ts, 1) >= camIdx && size(ts, 2) >= frameIdx
        t = ts(camIdx, frameIdx);
    elseif numel(ts) >= frameIdx
        t = ts(frameIdx);
    end
end
end

function sync = frameSyncMs(session, frameIdx)
sync = NaN;
if isfield(session, 'log') && isfield(session.log, 'syncMs') && numel(session.log.syncMs) >= frameIdx
    sync = session.log.syncMs(frameIdx);
end
end
