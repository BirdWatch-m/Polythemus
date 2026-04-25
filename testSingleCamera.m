% TESTSINGLCAMERA  Single-camera smoke test for the detection pipeline.
%
%   Run this script (F5) to verify the full per-camera pipeline works
%   before purchasing additional cameras or running extrinsic calibration.
%
%   Tests in order:
%     1. Camera acquisition and ring buffer
%     2. Sky mask drawing (skipped if mask file already exists)
%     3. Background subtraction and blob detection
%     4. Live display overlay
%     5. Session logging and save
%
%   Runs a per-stage latency benchmark before the main loop, then tracks
%   per-stage timing, FPS, jitter, blob size, and ring buffer fill during
%   the session. A full diagnostic report is printed at the end.
%   Press Q in the live display window to end the session.
%
%   USER INPUTS — edit these before running
camIdx   = 1;       % which camera to use
nSeconds    = 90;     % how long to run before auto-stopping (0 = Q key only)
benchFrames = 20;     % frames used for pre-loop benchmark
saveFrames  = false;  % save raw frames for offline replay
frameDir    = 'output/frames/';
 
% -------------------------------------------------------------------------
 
cfg   = buildConfig();
cfg.N = 1;   % override to single camera for this test
 
% Draw sky mask if not already saved.
if ~isfile(cfg.skyMaskFile)
    fprintf('No sky mask found. Drawing now...\n');
    drawSkyMasks(cfg);
end
 
% Initialise system (skips calibration check for single-camera test).
fprintf('Initialising...\n');
 
cam = webcam(camIdx);
cam.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));

% Warm-up: allow camera auto-exposure to stabilise before benchmarking.
fprintf('Camera warming up (3s)...\n');
warnState = warning('off', 'all');   % suppress timeout warnings during warmup
tWarm = tic();
while toc(tWarm) < 3.0
    try
        snapshot(cam);
    catch
    end
end
warning(warnState);
fprintf('Warmup done.\n');
 
H = cfg.resolution(2);
W = cfg.resolution(1);
 
ringBuf    = {zeros(H, W, cfg.ringBufLen, 'uint8')};
ringIdx    = 1;
bgMedian   = {zeros(H, W, 'double')};
fgDetector = vision.ForegroundDetector('NumGaussians', 3, ...
                                        'NumTrainingFrames', 60, ...
                                        'LearningRate', 0.005);
loaded     = load(cfg.skyMaskFile);
skyMask    = {loaded.skyMasks{1}};
 
% -------------------------------------------------------------------------
% PRE-LOOP BENCHMARK
% -------------------------------------------------------------------------
fprintf('\n--- Per-stage latency benchmark (%d frames) ---\n', benchFrames);
 
benchFrame    = snapshot(cam);
benchGray     = rgb2gray(benchFrame);
benchRingBuf  = zeros(H, W, cfg.ringBufLen, 'uint8');
benchBg       = zeros(H, W, 'double');
benchFd       = vision.ForegroundDetector('NumGaussians', 3, ...
                                           'NumTrainingFrames', 60, ...
                                           'LearningRate', 0.005);
benchMask     = true(H, W);
benchFig      = figure('Visible','off');
 
t_snapshot  = zeros(1, benchFrames);
t_rgb2gray  = zeros(1, benchFrames);
t_median    = zeros(1, benchFrames);
t_gmm       = zeros(1, benchFrames);
t_reprops   = zeros(1, benchFrames);
t_display   = zeros(1, benchFrames);
 
for b = 1:benchFrames
    t0 = tic;
    try
        benchFrame = snapshot(cam);
        t_snapshot(b) = toc(t0)*1000;
    catch
        t_snapshot(b) = NaN;
        continue;
    end          
    t_snapshot(b) = toc(t0)*1000;
    t0 = tic; benchGray  = rgb2gray(benchFrame);   t_rgb2gray(b) = toc(t0)*1000;
    t0 = tic; median(benchRingBuf, 3);             t_median(b)   = toc(t0)*1000;
    t0 = tic; step(benchFd, benchGray);            t_gmm(b)      = toc(t0)*1000;
    t0 = tic; regionprops(benchMask, 'Centroid', 'Area', ...
              'BoundingBox','MajorAxisLength','MinorAxisLength');
                                                   t_reprops(b)  = toc(t0)*1000;
    t0 = tic; image(benchFrame); drawnow limitrate; t_display(b) = toc(t0)*1000;
end
close(benchFig);
 
fprintf('  snapshot():      %5.1fms\n', nanmean(t_snapshot));
fprintf('  rgb2gray():      %5.1fms\n', nanmean(t_rgb2gray));
fprintf('  median(buf):     %5.1fms\n', nanmean(t_median));
fprintf('  GMM step():      %5.1fms\n', nanmean(t_gmm));
fprintf('  regionprops():   %5.1fms\n', nanmean(t_reprops));
fprintf('  image+drawnow:   %5.1fms\n', nanmean(t_display));
fprintf('  --- total est:   %5.1fms  (~%.0f fps theoretical)\n', ...
        nanmean(t_snapshot+t_rgb2gray+t_median+t_gmm+t_reprops+t_display), ...
        1000/nanmean(t_snapshot+t_rgb2gray+t_median+t_gmm+t_reprops+t_display));
fprintf('  snapshot jitter: %5.1fms std\n\n', nanstd(t_snapshot));
 
log.timestamps    = [];
log.syncFlags     = [];
log.nBlobs        = [];
log.blobCentroids = {};
 
% Per-stage loop timing accumulators.
t_loop_snapshot = [];
t_loop_detect   = [];
t_loop_display  = [];
t_loop_total    = [];
prevTimestamp   = 0;
interFrameTimes = [];
 
if saveFrames && ~isfolder(frameDir)
    mkdir(frameDir);
end
 
tStart    = tic();
tStop     = nSeconds > 0;
frameCount = 0;
 
% Set up Q key listener on figure.
fig = figure('Name', 'BirdTracker — live', 'NumberTitle', 'off', ...
             'KeyPressFcn', @(~,e) setappdata(gcf,'lastKey',e.Key));
setappdata(fig, 'lastKey', '');
 
fprintf('Running. Press Q to stop.\n\n');
 
while ishandle(fig)
 
    % 1. Acquire.
    t0 = tic;
    frame     = snapshot(cam);
    timestamp = toc(tStart);
    t_loop_snapshot(end+1) = toc(t0)*1000;
 
    % Inter-frame interval for jitter tracking.
    if prevTimestamp > 0
        interFrameTimes(end+1) = (timestamp - prevTimestamp)*1000;
    end
    prevTimestamp = timestamp;
 
    % 2. Update ring buffer.
    ringBuf{1}(:,:,ringIdx) = rgb2gray(frame);
    ringIdx = mod(ringIdx, cfg.ringBufLen) + 1;
    ringFillPct = min(100, round(100 * frameCount / cfg.medianBufLen));
 
    % 3. Detect.
    t0 = tic;
    frames = {frame};
    state.bgMedian    = bgMedian;
    state.ringBuf     = ringBuf;
    state.fgDetectors = {fgDetector};
    state.skyMask     = skyMask;
 
    [blobs, state] = detectBlobs(frames, state, cfg);
    bgMedian       = state.bgMedian;
    t_loop_detect(end+1) = toc(t0)*1000;
 
    % 4. Display.
    t0 = tic;
    renderFrame(frames, blobs, cfg);
    t_loop_display(end+1) = toc(t0)*1000;
 
    t_loop_total(end+1) = t_loop_snapshot(end) + t_loop_detect(end) + t_loop_display(end);
 
    % 5. Save raw frame.
    if saveFrames
        imwrite(frame, sprintf('%sframe_%06d.png', frameDir, frameCount));
    end
 
    % 6. Log.
    log = logFrame(log, blobs, [timestamp], true, cfg);
 
    frameCount = frameCount + 1;
 
    % Console update every 30 frames.
    if mod(frameCount, 30) == 0
        rollingFps = 1000 / mean(t_loop_total(max(1,end-29):end));
 
        % Blob area stats this frame.
        if ~isempty(blobs{1})
            areas   = [blobs{1}.area];
            areaStr = sprintf('area min/mean/max: %.0f/%.0f/%.0f px²', ...
                              min(areas), mean(areas), max(areas));
        else
            areaStr = 'no blobs';
        end
 
        fprintf('t=%5.1fs | fps:%4.1f | buf:%3d%% | blobs:%3d | %s\n', ...
                timestamp, rollingFps, ringFillPct, numel(blobs{1}), areaStr);
    end
 
    % Stop conditions.
    if tStop && timestamp >= nSeconds
        fprintf('Time limit reached.\n');
        break;
    end
    if strcmp(getappdata(fig,'lastKey'), 'q')
        fprintf('Q pressed.\n');
        break;
    end
 
end
 
if ishandle(fig), close(fig); end
clear cam;
 
% -------------------------------------------------------------------------
% DIAGNOSTIC REPORT
% -------------------------------------------------------------------------
fprintf('\n--- Session diagnostics ---\n');
fprintf('Frames recorded:   %d\n', frameCount);
fprintf('Duration:          %.1fs\n', log.timestamps(end) - log.timestamps(1));
fprintf('\nAchieved FPS:\n');
fprintf('  mean:            %.1f fps\n', 1000/mean(t_loop_total));
fprintf('  min:             %.1f fps\n', 1000/max(t_loop_total));
fprintf('  max:             %.1f fps\n', 1000/min(t_loop_total));
fprintf('\nPer-stage timing (mean):\n');
fprintf('  snapshot():      %5.1fms\n', mean(t_loop_snapshot));
fprintf('  detect pipeline: %5.1fms\n', mean(t_loop_detect));
fprintf('  display:         %5.1fms\n', mean(t_loop_display));
fprintf('  total:           %5.1fms\n', mean(t_loop_total));
if ~isempty(interFrameTimes)
fprintf('\nInter-frame interval:\n');
fprintf('  mean:            %5.1fms\n', mean(interFrameTimes));
fprintf('  std (jitter):    %5.1fms\n', std(interFrameTimes));
fprintf('  max gap:         %5.1fms\n', max(interFrameTimes));
end
fprintf('\nBlob statistics:\n');
fprintf('  mean/frame:      %.1f\n',  mean(log.nBlobs));
fprintf('  max/frame:       %.0f\n',  max(log.nBlobs));
fprintf('\nRing buffer:       filled to %.0f%% at end\n', ...
        min(100, round(100*frameCount/cfg.medianBufLen)));
fprintf('\nBottleneck:        %s\n', identifyBottleneck( ...
        mean(t_loop_snapshot), mean(t_loop_detect), mean(t_loop_display)));
 
saveSession(log, cfg);
 
% -------------------------------------------------------------------------
function s = identifyBottleneck(tSnap, tDet, tDisp)
[~, idx] = max([tSnap, tDet, tDisp]);
names = {'snapshot() — hardware limited', ...
         'detect pipeline — tune thresholds or amortise median', ...
         'display — try drawnow limitrate or reduce display rate'};
s = names{idx};
end