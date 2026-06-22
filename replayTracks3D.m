% REPLAYTRACKS3D  Render a synchronized 3D replay of confirmed tracks.
%
%   Script. Run processRecording (or processRecordingTuned) first, then set
%   recordingDir/resultsFileName below and run with F5. The output uses the same
%   absolute frame numbers as replaySession/replaySessionTuned, so the 3D video
%   can be placed next to the annotated camera videos in a presentation.
%
%   Outputs in the recording folder:
%     tracks3D_replay_frame_NNNNNN.png       publication screenshot
%     tracks3D_replay.mp4                    PowerPoint-friendly video
%     tracks3D_replay.avi                    AVI matching replaySession style
%
%   The plotted coordinate frame is a presentation transform of the camera-1
%   world frame: X = right, Y = forward depth, Z = up. Values are still metres.
%
%   See also: processRecording, processRecordingTuned, replaySession

clc; close all; clear;
addpath(genpath(fileparts(mfilename('fullpath'))));

% =========================================================================
% USER INPUTS
% =========================================================================
recordingDir    = 'output/recordings/20260621_134908';
resultsFileName = 'results.mat';     % 'results.mat' or 'results_tuned.mat'
playbackFps     = 30;                % match replaySession / replaySessionTuned

% Absolute frame range in the original recording. [] uses the whole results
% window; for results_tuned.mat this means results.frameIndex.
frameRange      = [];

% Screenshot frame as an absolute recording frame. [] picks the middle frame
% among frames with confirmed tracks.
screenshotFrame = [];

trailSeconds    = 2.5;               % length of visible track trails
axisLimits      = [];                % [xmin xmax; forwardMin forwardMax; upMin upMax], [] = auto
viewAzEl        = [-42 22];          % [azimuth elevation]
figureSize      = [1280 720];        % output canvas in pixels

writePng        = true;
writeMp4        = true;
writeAvi        = true;
outputStem      = '';                % '' -> recordingDir/tracks3D_replay[_tuned]

% =========================================================================
% LOAD SESSION + RESULTS
% =========================================================================
sessionFile = fullfile(recordingDir, 'session.mat');
if ~isfile(sessionFile)
    error('replayTracks3D:noSession', 'session.mat not found in %s', recordingDir);
end
loaded  = load(sessionFile);
session = loaded.session;
cfg     = session.cfg;

resultsFile = fullfile(recordingDir, resultsFileName);
if ~isfile(resultsFile)
    error('replayTracks3D:noResults', '%s not found. Run processRecording first.', resultsFile);
end
resLoaded = load(resultsFile);
results   = resLoaded.results;
if ~isfield(results, 'trackPositions')
    error('replayTracks3D:noTracks', ...
          '%s does not contain full-pipeline trackPositions. Re-run with runMode=''full''.', resultsFile);
end
results = ensureTrackIds(results, resultsFile);

calibration = loadRecordingCalibration(recordingDir, session, cfg);

[frameIndices, keep] = resolveFrameWindow(results, session, frameRange);
trackPositions = results.trackPositions(keep);
trackIds       = results.trackIds(keep);
nFrames        = numel(frameIndices);
if nFrames == 0
    error('replayTracks3D:emptyWindow', 'No result frames match the requested frameRange.');
end

if isfield(results, 'nConfirmed')
    nConfirmed = results.nConfirmed(keep);
else
    nConfirmed = cellfun(@(p) size(p, 1), trackPositions);
end
if isfield(results, 'meanReprErr')
    meanReprErr = results.meanReprErr(keep);
else
    meanReprErr = nan(1, nFrames);
end

frameTimes = arrayfun(@(k) meanFrameTime(session, k), frameIndices);
syncMs     = arrayfun(@(k) frameSyncMs(session, k), frameIndices);
series     = buildTrackSeries(trackIds, trackPositions);

allTrackPos = collectAllPositions(trackPositions);
if isempty(allTrackPos)
    error('replayTracks3D:noConfirmedTracks', ...
          'No confirmed track positions found in %s. Nothing useful to render.', resultsFile);
end

cameraCenters = getCameraCenters(calibration);
plotTrackPos  = toPlotCoords(allTrackPos);
plotCameras   = toPlotCoords(cameraCenters);
if isempty(axisLimits)
    axisLimits = computeAxisLimits(plotTrackPos, plotCameras);
end

if isempty(outputStem)
    outputStem = defaultOutputStem(recordingDir, resultsFileName);
else
    outputStem = fullfile(recordingDir, outputStem);
end

screenshotKi = chooseScreenshotIndex(frameIndices, nConfirmed, screenshotFrame);
trailFrames  = max(1, round(trailSeconds * cfg.fps));

fprintf('3D replay: %s\n', resultsFile);
fprintf('Frames:    %d - %d (%d rendered)\n', frameIndices(1), frameIndices(end), nFrames);
fprintf('Tracks:    %d confirmed track id(s)\n', numel(series));
fprintf('Outputs:   %s.[png/mp4/avi]\n', outputStem);

% =========================================================================
% FIGURE SETUP
% =========================================================================
fig = figure('Name', 'BirdTracker 3D track replay', 'NumberTitle', 'off', ...
             'Color', 'white', 'Units', 'pixels', ...
             'Position', [100 100 figureSize(1) figureSize(2)]);
ax = axes('Parent', fig);
hold(ax, 'on');
setupAxes(ax, axisLimits, viewAzEl);
drawCameraRig(ax, calibration, cfg, axisLimits);

trackHandles = createTrackHandles(ax, series);
statusText = annotation(fig, 'textbox', [0.015 0.018 0.50 0.045], ...
    'String', '', 'EdgeColor', 'none', 'Color', [0.15 0.15 0.15], ...
    'FontName', 'Arial', 'FontSize', 10, 'Interpreter', 'none');

% Save the thesis screenshot before video rendering. The same draw function is
% used for both outputs, so the still and video use identical styling.
drawReplayFrame(ax, trackHandles, series, screenshotKi, trailFrames, ...
                frameIndices, frameTimes, syncMs, nConfirmed, meanReprErr, statusText);
drawnow;
if writePng
    pngOut = sprintf('%s_frame_%06d.png', outputStem, frameIndices(screenshotKi));
    exportFigurePng(fig, pngOut, 300);
    fprintf('Saved PNG: %s\n', pngOut);
end

% =========================================================================
% VIDEO RENDER
% =========================================================================
writers = openVideoWriters(outputStem, playbackFps, writeMp4, writeAvi);
try
    for ki = 1:nFrames
        drawReplayFrame(ax, trackHandles, series, ki, trailFrames, ...
                        frameIndices, frameTimes, syncMs, nConfirmed, meanReprErr, statusText);
        drawnow limitrate;
        if ~isempty(writers)
            fr = getframe(fig);
            for w = 1:numel(writers)
                writeVideo(writers{w}, fr);
            end
        end
        if mod(ki, 100) == 0 || ki == nFrames
            fprintf('  rendered %d / %d frames\n', ki, nFrames);
        end
    end
catch ME
    closeVideoWriters(writers);
    rethrow(ME);
end
closeVideoWriters(writers);

fprintf('Done.\n');

% =========================================================================
% LOCAL HELPERS
% =========================================================================
function results = ensureTrackIds(results, resultsFile)
if isfield(results, 'trackIds')
    return;
end
n = numel(results.trackPositions);
results.trackIds = cell(1, n);
maxRows = 0;
for k = 1:n
    maxRows = max(maxRows, size(results.trackPositions{k}, 1));
end
if maxRows > 1
    error('replayTracks3D:missingTrackIds', ...
          ['%s has multiple tracks but no results.trackIds field. ' ...
           'Re-run processRecording/processRecordingTuned with the updated script.'], resultsFile);
end
for k = 1:n
    if isempty(results.trackPositions{k})
        results.trackIds{k} = zeros(0, 1);
    else
        results.trackIds{k} = 1;
    end
end
warning('replayTracks3D:legacyResults', ...
        ['%s has no results.trackIds field. Using a legacy single-track fallback; ' ...
         're-run processing for publication outputs.'], resultsFile);
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
    error('replayTracks3D:noCalibration', ...
          'No calibration snapshot found in %s or cfg.calFile.', recordingDir);
end
end

function [frameIndices, keep] = resolveFrameWindow(results, session, frameRange)
if isfield(results, 'frameIndex')
    allFrameIndices = results.frameIndex(:).';
else
    allFrameIndices = 1:numel(results.trackPositions);
end
if isempty(frameRange)
    keep = true(size(allFrameIndices));
else
    if numel(frameRange) ~= 2 || frameRange(1) > frameRange(2)
        error('replayTracks3D:badFrameRange', 'frameRange must be [] or [firstFrame lastFrame].');
    end
    keep = allFrameIndices >= frameRange(1) & allFrameIndices <= frameRange(2);
end
frameIndices = allFrameIndices(keep);
if ~isempty(frameIndices) && isfield(session, 'nFrames')
    bad = frameIndices < 1 | frameIndices > session.nFrames;
    if any(bad)
        error('replayTracks3D:frameOutOfRange', 'Requested frame index is outside session.nFrames.');
    end
end
end

function series = buildTrackSeries(trackIds, trackPositions)
allIds = [];
for k = 1:numel(trackIds)
    allIds = [allIds; trackIds{k}(:)]; %#ok<AGROW>
end
allIds = unique(allIds(:).');
allIds = allIds(~isnan(allIds));
series = repmat(struct('id', [], 'localFrame', [], 'pos', [], 'plotPos', [], 'color', []), 1, 0);
colors = trackPalette(numel(allIds));
for j = 1:numel(allIds)
    id = allIds(j);
    localFrame = [];
    pos = [];
    for k = 1:numel(trackIds)
        ids = trackIds{k}(:);
        p   = trackPositions{k};
        if isempty(ids), continue; end
        if size(p, 1) ~= numel(ids)
            error('replayTracks3D:trackShapeMismatch', ...
                  'trackIds and trackPositions disagree at result frame %d.', k);
        end
        rows = find(ids == id);
        if isempty(rows), continue; end
        localFrame = [localFrame; repmat(k, numel(rows), 1)]; %#ok<AGROW>
        pos = [pos; p(rows, :)]; %#ok<AGROW>
    end
    s.id = id;
    s.localFrame = localFrame;
    s.pos = pos;
    s.plotPos = toPlotCoords(pos);
    s.color = colors(j, :);
    series(end+1) = s; %#ok<AGROW>
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

function handles = createTrackHandles(ax, series)
handles = repmat(struct('trail', [], 'head', [], 'label', []), 1, numel(series));
for j = 1:numel(series)
    c = series(j).color;
    trailColor = 0.55 * c + 0.45 * [1 1 1];
    handles(j).trail = plot3(ax, nan, nan, nan, '-', ...
        'Color', trailColor, 'LineWidth', 2.4);
    handles(j).head = scatter3(ax, nan, nan, nan, 70, 'filled', ...
        'MarkerFaceColor', c, 'MarkerEdgeColor', [0.05 0.05 0.05], ...
        'LineWidth', 0.7);
    handles(j).label = text(ax, nan, nan, nan, '', ...
        'FontName', 'Arial', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Color', c, 'BackgroundColor', 'white', 'Margin', 2, ...
        'Clipping', 'on');
    set([handles(j).trail, handles(j).head, handles(j).label], 'Visible', 'off');
end
end

function drawReplayFrame(ax, handles, series, ki, trailFrames, frameIndices, frameTimes, ...
                         syncMs, nConfirmed, meanReprErr, statusText)
trailStart = max(1, ki - trailFrames + 1);
for j = 1:numel(series)
    inTrail = series(j).localFrame >= trailStart & series(j).localFrame <= ki;
    current = series(j).localFrame == ki;
    if any(inTrail)
        pts = series(j).plotPos(inTrail, :);
        set(handles(j).trail, 'XData', pts(:,1), 'YData', pts(:,2), 'ZData', pts(:,3), 'Visible', 'on');
    else
        set(handles(j).trail, 'XData', nan, 'YData', nan, 'ZData', nan, 'Visible', 'off');
    end
    if any(current)
        p = series(j).plotPos(find(current, 1, 'last'), :);
        set(handles(j).head, 'XData', p(1), 'YData', p(2), 'ZData', p(3), 'Visible', 'on');
        set(handles(j).label, 'Position', p + [0.4 0.4 0.4], ...
            'String', sprintf('ID %d', series(j).id), 'Visible', 'on');
    else
        set([handles(j).head, handles(j).label], 'Visible', 'off');
    end
end

reprText = 'n/a';
if ~isnan(meanReprErr(ki))
    reprText = sprintf('%.2f px', meanReprErr(ki));
end
syncText = 'n/a';
if ~isnan(syncMs(ki))
    syncText = sprintf('%.1f ms', syncMs(ki));
end
title(ax, sprintf('3D track replay - frame %d - t = %.2f s', frameIndices(ki), frameTimes(ki)), ...
      'FontName', 'Arial', 'FontSize', 15, 'FontWeight', 'bold');
statusText.String = sprintf('confirmed: %d   reprojection error: %s   sync: %s', ...
                            nConfirmed(ki), reprText, syncText);
end

function setupAxes(ax, axisLimits, viewAzEl)
set(ax, 'FontName', 'Arial', 'FontSize', 11, 'LineWidth', 0.9, ...
    'Color', [0.985 0.985 0.985], 'GridColor', [0.70 0.70 0.70], ...
    'MinorGridColor', [0.86 0.86 0.86], 'Projection', 'perspective');
grid(ax, 'on');
box(ax, 'on');
view(ax, viewAzEl(1), viewAzEl(2));
xlabel(ax, 'X right from camera 1 (m)', 'FontName', 'Arial', 'FontSize', 12);
ylabel(ax, 'Z forward from camera 1 (m)', 'FontName', 'Arial', 'FontSize', 12);
zlabel(ax, 'Y up from camera 1 (m)', 'FontName', 'Arial', 'FontSize', 12);
xlim(ax, axisLimits(1,:));
ylim(ax, axisLimits(2,:));
zlim(ax, axisLimits(3,:));
daspect(ax, [1 1 1]);
end

function drawCameraRig(ax, calibration, cfg, axisLimits)
centers = getCameraCenters(calibration);
if isempty(centers), return; end
centersPlot = toPlotCoords(centers);
axisSpan = max(axisLimits(:,2) - axisLimits(:,1));
plot3(ax, centersPlot(:,1), centersPlot(:,2), centersPlot(:,3), 'k--', 'LineWidth', 1.2);
for i = 1:size(centersPlot, 1)
    scatter3(ax, centersPlot(i,1), centersPlot(i,2), centersPlot(i,3), 95, 's', ...
        'MarkerFaceColor', [0.08 0.08 0.08], 'MarkerEdgeColor', 'white', 'LineWidth', 0.8);
    text(ax, centersPlot(i,1), centersPlot(i,2), centersPlot(i,3) + 0.5, ...
        sprintf('Cam %d', i), 'FontName', 'Arial', 'FontSize', 9, ...
        'FontWeight', 'bold', 'Color', [0.08 0.08 0.08], ...
        'BackgroundColor', 'white', 'Margin', 2, 'Clipping', 'on');
    drawCameraFrustum(ax, calibration, cfg, i, max(1.0, axisSpan * 0.08));
end
end

function drawCameraFrustum(ax, calibration, cfg, camIdx, len)
if ~isfield(calibration, 'intrinsics') || numel(calibration.intrinsics) < camIdx
    return;
end
if ~isfield(calibration, 'R') || ~isfield(calibration, 't')
    return;
end
K = calibration.intrinsics{camIdx}.IntrinsicMatrix.';
if isfield(cfg, 'resolution')
    W = cfg.resolution(1);
    H = cfg.resolution(2);
elseif isprop(calibration.intrinsics{camIdx}, 'ImageSize')
    H = calibration.intrinsics{camIdx}.ImageSize(1);
    W = calibration.intrinsics{camIdx}.ImageSize(2);
else
    W = 1280;
    H = 720;
end
corners = [1 1; W 1; W H; 1 H];
R = calibration.R{camIdx};
t = calibration.t{camIdx}(:);
C = -R.' * t;
frustumWorld = zeros(4, 3);
for c = 1:4
    rayCam = K \ [corners(c,1); corners(c,2); 1];
    rayWorld = R.' * rayCam;
    rayWorld = rayWorld / norm(rayWorld);
    frustumWorld(c,:) = (C + len * rayWorld).';
end
Cplot = toPlotCoords(C.');
Fplot = toPlotCoords(frustumWorld);
for c = 1:4
    plot3(ax, [Cplot(1) Fplot(c,1)], [Cplot(2) Fplot(c,2)], [Cplot(3) Fplot(c,3)], ...
        '-', 'Color', [0.18 0.18 0.18], 'LineWidth', 0.8);
end
order = [1 2 3 4 1];
plot3(ax, Fplot(order,1), Fplot(order,2), Fplot(order,3), '-', ...
    'Color', [0.18 0.18 0.18], 'LineWidth', 0.8);
end

function centers = getCameraCenters(calibration)
if ~isfield(calibration, 'R') || ~isfield(calibration, 't')
    centers = zeros(0, 3);
    return;
end
N = numel(calibration.R);
centers = zeros(N, 3);
for i = 1:N
    centers(i,:) = (-calibration.R{i}.' * calibration.t{i}(:)).';
end
end

function plotPts = toPlotCoords(xyz)
if isempty(xyz)
    plotPts = zeros(0, 3);
    return;
end
plotPts = [xyz(:,1), xyz(:,3), -xyz(:,2)];
end

function lim = computeAxisLimits(trackPlot, cameraPlot)
data = [trackPlot; cameraPlot];
mins = min(data, [], 1);
maxs = max(data, [], 1);
span = maxs - mins;
minSpan = 4.0;
for d = 1:3
    if span(d) < minSpan
        c = 0.5 * (mins(d) + maxs(d));
        mins(d) = c - minSpan / 2;
        maxs(d) = c + minSpan / 2;
        span(d) = minSpan;
    end
end
pad = max(0.10 * span, 1.0);
lim = [mins(1)-pad(1), maxs(1)+pad(1);
       mins(2)-pad(2), maxs(2)+pad(2);
       mins(3)-pad(3), maxs(3)+pad(3)];
end

function xyz = collectAllPositions(trackPositions)
xyz = zeros(0, 3);
for k = 1:numel(trackPositions)
    p = trackPositions{k};
    if ~isempty(p)
        xyz = [xyz; p]; %#ok<AGROW>
    end
end
end

function ki = chooseScreenshotIndex(frameIndices, nConfirmed, screenshotFrame)
if isempty(screenshotFrame)
    active = find(nConfirmed > 0);
    if isempty(active)
        ki = ceil(numel(frameIndices) / 2);
    else
        ki = active(round(numel(active) / 2));
    end
else
    ki = find(frameIndices == screenshotFrame, 1);
    if isempty(ki)
        error('replayTracks3D:badScreenshotFrame', ...
              'screenshotFrame %d is not in the selected result window.', screenshotFrame);
    end
end
end

function t = meanFrameTime(session, frameIdx)
t = NaN;
if ~isfield(session, 'log'), return; end
if isfield(session.log, 'cameraTimestamps') && size(session.log.cameraTimestamps, 2) >= frameIdx
    vals = session.log.cameraTimestamps(:, frameIdx);
elseif isfield(session.log, 'timestamps')
    ts = session.log.timestamps;
    if ismatrix(ts) && size(ts, 2) >= frameIdx
        if size(ts, 1) > 1
            vals = ts(:, frameIdx);
        else
            vals = ts(frameIdx);
        end
    elseif numel(ts) >= frameIdx
        vals = ts(frameIdx);
    else
        vals = NaN;
    end
else
    vals = NaN;
end
vals = vals(~isnan(vals));
if ~isempty(vals)
    t = mean(vals);
end
end

function sync = frameSyncMs(session, frameIdx)
sync = NaN;
if isfield(session, 'log') && isfield(session.log, 'syncMs') && numel(session.log.syncMs) >= frameIdx
    sync = session.log.syncMs(frameIdx);
end
end

function stem = defaultOutputStem(recordingDir, resultsFileName)
[~, base] = fileparts(resultsFileName);
if strcmpi(base, 'results')
    tag = 'replay';
elseif startsWith(lower(base), 'results_')
    tag = ['replay_' base(numel('results_')+1:end)];
else
    tag = base;
end
stem = fullfile(recordingDir, ['tracks3D_' tag]);
end

function writers = openVideoWriters(outputStem, fps, writeMp4, writeAvi)
writers = {};
if writeMp4
    mp4Out = [outputStem '.mp4'];
    try
        vw = VideoWriter(mp4Out, 'MPEG-4');
        vw.FrameRate = fps;
        if isprop(vw, 'Quality'), vw.Quality = 95; end
        open(vw);
        writers{end+1} = vw;
        fprintf('Writing MP4: %s\n', mp4Out);
    catch ME
        warning('replayTracks3D:mp4Unavailable', 'Could not open MP4 writer: %s', ME.message);
    end
end
if writeAvi
    aviOut = [outputStem '.avi'];
    vw = VideoWriter(aviOut, 'Motion JPEG AVI');
    vw.FrameRate = fps;
    if isprop(vw, 'Quality'), vw.Quality = 95; end
    open(vw);
    writers{end+1} = vw;
    fprintf('Writing AVI: %s\n', aviOut);
end
end

function closeVideoWriters(writers)
for w = 1:numel(writers)
    try
        close(writers{w});
    catch
    end
end
end

function exportFigurePng(fig, outFile, dpi)
try
    exportgraphics(fig, outFile, 'Resolution', dpi);
catch
    print(fig, outFile, '-dpng', sprintf('-r%d', dpi));
end
end