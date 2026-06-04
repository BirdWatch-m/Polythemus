% MAIN  Multi-camera detection smoke test (no calibration required).
%
%   Captures from all cfg.N cameras, runs the per-camera detection pipeline
%   (background subtraction -> blob gating), and shows live overlays side by
%   side. Purpose: validate the multi-camera DETECTION path end to end on real
%   sky — especially BUG-1 (per-camera background counter) and cfg.camIndices.
%
%   Sky masks are full-frame here so testing is quick (no drawing). Draw real
%   masks with drawSkyMasks for actual operation.
%
%   Press Q (with the live figure focused) to stop.

clc; close all; clear;

cfg = buildConfig();

N = cfg.N;
H = cfg.resolution(2);
W = cfg.resolution(1);

% 1) Extrinsic calibration (re-runs every test — fine for now).
calibrateExtrinsics(cfg, {'calibration/intrinsics_MY1_720.mat', ...
                          'calibration/intrinsics_LG1_720.mat'});

% -------------------------------------------------------------------------
% Open cameras. Logical camera i -> physical webcamlist index (cfg.camIndices).
% -------------------------------------------------------------------------
cams = cell(1, N);
for i = 1:N
    cams{i} = webcam(cfg.camIndices(i));
    cams{i}.Resolution = sprintf('%dx%d', W, H);
end

% Warm up each camera — the first frames after opening can time out / be noisy.
warnState = warning('off', 'all');
for i = 1:N
    tWarm = tic;
    while toc(tWarm) < 2.0
        try, snapshot(cams{i}); catch, end
    end
end
warning(warnState);

% -------------------------------------------------------------------------
% Build per-camera detection state manually (no initSystem, so no extrinsic
% calibration file is needed for a detection-only test).
% -------------------------------------------------------------------------
state.ringBuf             = cell(1, N);
state.ringIdx             = ones(1, N);
state.bgMedian            = cell(1, N);
state.bgFramesSinceUpdate = zeros(1, N);      % per-camera counter (BUG-1)
state.fgDetectors         = cell(1, N);
state.skyMask             = cell(1, N);
for i = 1:N
    state.ringBuf{i}     = zeros(H, W, cfg.ringBufLen, 'uint8');
    state.bgMedian{i}    = zeros(H, W, 'double');
    state.fgDetectors{i} = vision.ForegroundDetector( ...
        'NumGaussians', 3, 'NumTrainingFrames', 60, 'LearningRate', 0.005);
    state.skyMask{i}     = true(H, W);        % full-frame mask — no drawing
end

% -------------------------------------------------------------------------
% Q-to-stop figure. Named to match renderFrame so it reuses this figure and
% the keypress handler stays attached.
% -------------------------------------------------------------------------
fig = figure('Name', 'BirdTracker — live', 'NumberTitle', 'off', ...
             'KeyPressFcn', @(~,e) setappdata(gcf, 'lastKey', e.Key));
setappdata(fig, 'lastKey', '');

fprintf('Running %d-camera detection at %dx%d. Press Q to stop.\n', N, W, H);
tStart     = tic;
frameCount = 0;
accAcq = 0; accDet = 0; accDisp = 0;   % per-stage time accumulators (ms); reset each report

while ishandle(fig)

    % 1. Acquire one frame per camera (as close in time as snapshot allows).
    tA = tic;
    [frames, timestamps] = acquireFrames(cams, tStart, cfg);
    accAcq = accAcq + toc(tA) * 1000;

    % 2-3. Ring buffer update + detection (per-camera bgMedian + bgFramesSinceUpdate, BUG-1).
    %      updateRingBuf returns the grayscale frames so detection reuses them (PERF-1).
    tD = tic;
    [state.ringBuf, state.ringIdx, grayFrames] = ...
        updateRingBuf(state.ringBuf, state.ringIdx, frames, cfg);
    [blobs, state] = detectBlobs(grayFrames, state, cfg);
    accDet = accDet + toc(tD) * 1000;

    % 4. Display side-by-side feeds with overlays.
    tR = tic;
    renderFrame(frames, blobs, cfg);
    accDisp = accDisp + toc(tR) * 1000;

    frameCount = frameCount + 1;

    % Console report every 30 frames.
    if mod(frameCount, 30) == 0
        counts  = cellfun(@numel, blobs);                       % blobs per camera
        syncMs  = (max(timestamps) - min(timestamps)) * 1000;   % inter-camera lag
        fillPct = min(100, round(100 * frameCount / cfg.medianBufLen));
        loopMs  = (accAcq + accDet + accDisp) / 30;             % avg total per frame
        fprintf(['t=%5.1fs | frame %4d | buf:%3d%% | sync:%4.1fms | blobs/cam: %s\n' ...
                 '         ms/frame (avg 30): acquire %.1f | detect %.1f | render %.1f | total %.1f (~%.1f fps)\n'], ...
                toc(tStart), frameCount, fillPct, syncMs, mat2str(counts), ...
                accAcq/30, accDet/30, accDisp/30, loopMs, 1000/loopMs);
        accAcq = 0; accDet = 0; accDisp = 0;
    end

    if strcmp(getappdata(fig, 'lastKey'), 'q'), break; end
end

if ishandle(fig), close(fig); end
cams = {};   % release the webcam handles
fprintf('Stopped after %d frames.\n', frameCount);
