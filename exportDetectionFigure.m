% EXPORTDETECTIONFIGURE  Export the six-panel detection-stages figure (Fig 4.2).
%
%   Script. Set the USER INPUTS below and run with F5.  Reads a session
%   recorded by recordSession, replays the detection pipeline on one frame
%   of one camera with full background-model warmup, captures each
%   intermediate stage, and saves a six-panel PNG at 300 DPI.
%
%   The six panels are:
%     (a) Raw frame     — grayscale input
%     (b) Median diff   — |frame - bgMedian|, stretched for visibility
%     (c) GMM mask      — binary output of vision.ForegroundDetector
%     (d) AND + sky     — (median threshold AND gmm) AND sky region
%     (e) Morphology    — after imopen + imclose
%     (f) Gated blobs   — raw frame with blob bounding boxes in yellow
%
%   Set targetFrame = 0 to auto-select the first frame (after background
%   warmup) where at least one blob passes all gates.  Once you have
%   identified a good frame manually, set targetFrame to that number
%   so the script always exports the same frame.
%
%   See also: recordSession, processRecording, applyBackground, gateBlobs

clc; close all;

% =========================================================================
% USER INPUTS
% =========================================================================
recordingDir = 'output/recordings/REPLACE_WITH_SESSION_FOLDER';
camIdx       = 1;      % which camera to show (1 to cfg.N)
targetFrame  = 0;      % specific frame number to export; 0 = auto-select
outFile      = 'output/figures/fig_4_2_detection.png';

% =========================================================================
% LOAD SESSION
% =========================================================================
sessionFile = fullfile(recordingDir, 'session.mat');
if ~isfile(sessionFile)
    error('exportDetectionFigure:noSession', 'session.mat not found in: %s', recordingDir);
end
s   = load(sessionFile);
cfg = s.session.cfg;
H   = cfg.resolution(2);
W   = cfg.resolution(1);
nFrames = s.session.nFrames;

if camIdx < 1 || camIdx > cfg.N
    error('exportDetectionFigure:badCam', 'camIdx must be 1 to %d (cfg.N)', cfg.N);
end

fprintf('Session: %d frames, %d camera(s), %dx%d\n', nFrames, cfg.N, W, H);

% =========================================================================
% LOAD SKY MASK
% =========================================================================
if isfile(cfg.skyMaskFile)
    m       = load(cfg.skyMaskFile, 'skyMasks');
    skyMask = m.skyMasks{camIdx};
    fprintf('Sky mask loaded.\n');
else
    skyMask = true(H, W);
    fprintf('Sky mask not found — using full frame.\n');
end

readGray = @(k) readGrayFrame( ...
    fullfile(recordingDir, sprintf('cam%d', camIdx), sprintf('frame_%06d.tif', k)));

% =========================================================================
% AUTO-SELECT TARGET FRAME  (skip if targetFrame > 0)
% =========================================================================
warmupLen = cfg.medianBufLen;    % frames before bgMedian is meaningful
scanCap   = warmupLen + 300;     % give up after this frame (~10 s at 30 fps)

if targetFrame == 0
    fprintf('Scanning frames %d–%d for first frame with blobs...\n', ...
            warmupLen + 1, min(nFrames, scanCap));

    [rb, ri, bgM, fsu, fgD] = initState(H, W, cfg);
    targetFrame = -1;

    for k = 1:min(nFrames, scanCap)
        gk = readGray(k);
        [rb, ri, bgM, fsu] = advanceBuffer(rb, ri, bgM, fsu, gk, cfg);
        if cfg.useGMM
            gmmK = step(fgD, gk);
        end

        if k <= warmupLen, continue; end    % still warming up — skip blob check

        diffK = abs(int16(gk) - bgM);
        mMed  = diffK > cfg.medianFgThreshold;
        if cfg.useGMM
            mSky = mMed & gmmK & skyMask;
        else
            mSky = mMed & skyMask;
        end
        if cfg.useMorphology
            mSky = imopen(mSky, cfg.morphStrel);
            mSky = imclose(mSky, cfg.morphStrel);
        end

        st = regionprops(mSky, 'Area', 'MajorAxisLength', 'MinorAxisLength');
        if ~isempty(st)
            okA = [st.Area] >= cfg.minBlobArea & [st.Area] <= cfg.maxBlobArea;
            if any(okA)
                safeMin = max([st(okA).MinorAxisLength], 0.1);
                if any([st(okA).MajorAxisLength] ./ safeMin <= cfg.maxAspect)
                    targetFrame = k;
                    fprintf('Blobs found at frame %d.\n', k);
                    break;
                end
            end
        end
    end

    if targetFrame < 0
        error('exportDetectionFigure:noBlobs', ...
              ['No blobs found in frames %d–%d.\n' ...
               'Capture a session with a visible target, or increase scanCap.'], ...
              warmupLen + 1, scanCap);
    end
end

% =========================================================================
% FRESH WARMUP TO targetFrame - 1
%   Always done from scratch so the state is consistent regardless of whether
%   auto-scan ran above.  Cost: one extra pass of imread calls, acceptable
%   for an offline once-per-thesis export.
% =========================================================================
fprintf('Warming up (frames 1–%d)...\n', targetFrame - 1);
[rb, ri, bgM, fsu, fgD] = initState(H, W, cfg);

for k = 1:targetFrame - 1
    gk = readGray(k);
    [rb, ri, bgM, fsu] = advanceBuffer(rb, ri, bgM, fsu, gk, cfg);
    if cfg.useGMM
        step(fgD, gk);
    end
end

% =========================================================================
% CAPTURE INTERMEDIATES FOR targetFrame
% =========================================================================
fprintf('Capturing detection stages for frame %d, camera %d...\n', targetFrame, camIdx);
frameGray = readGray(targetFrame);

% Insert into ring buffer and refresh median before running detection —
% this matches the pipeline order in main (updateRingBuf before detectBlobs).
[rb, ~, bgM, ~] = advanceBuffer(rb, ri, bgM, fsu, frameGray, cfg); %#ok<ASGLU>

% Median model.
diffImg     = abs(int16(frameGray) - bgM);
mask_median = diffImg > cfg.medianFgThreshold;

% GMM model.
if cfg.useGMM
    mask_gmm      = step(fgD, frameGray);
    mask_combined = mask_median & mask_gmm;
else
    mask_gmm      = false(H, W);
    mask_combined = mask_median;
end

% AND with sky region.
mask_sky = mask_combined & skyMask;

% Morphological cleanup (computed here for display in panel e).
if cfg.useMorphology
    mask_morph = imopen(mask_sky, cfg.morphStrel);
    mask_morph = imclose(mask_morph, cfg.morphStrel);
else
    mask_morph = mask_sky;
end

% Blob gating via the production function.  Pass mask_sky (pre-morphology)
% with a full-frame sky mask so gateBlobs applies morphology once internally,
% exactly matching the live pipeline.
blobs = gateBlobs(mask_sky, true(H, W), cfg);
fprintf('  %d blob(s) detected.\n', numel(blobs));

% =========================================================================
% BUILD FIGURE
% =========================================================================
% Stretch the diff image so foreground pixels (above threshold) are clearly
% visible.  Max normalise to fill 0-255; cap with min to guard a black frame.
diffPeak    = double(max(diffImg(:)));
if diffPeak < 1, diffPeak = 1; end
diffDisplay = uint8(min(double(diffImg) * (255 / diffPeak), 255));

% Blob overlay: yellow bounding boxes on the raw frame.
blobFrame = repmat(frameGray, 1, 1, 3);
if ~isempty(blobs)
    bboxes    = vertcat(blobs.bbox);
    blobFrame = insertShape(blobFrame, 'Rectangle', bboxes, 'Color', 'yellow', 'LineWidth', 2);
end

if cfg.useGMM
    gmmImg   = uint8(mask_gmm) * 255;
    gmmLabel = '(c) GMM mask';
else
    gmmImg   = uint8(mask_median) * 255;
    gmmLabel = '(c) GMM disabled';
end

imgs   = {frameGray,  diffDisplay,          gmmImg, ...
          uint8(mask_sky)*255, uint8(mask_morph)*255, blobFrame};
labels = {'(a) Raw frame', '(b) Median diff', gmmLabel, ...
          '(d) AND + sky region', '(e) Morphology', '(f) Gated blobs'};

fig = figure('Color', 'white', 'Units', 'inches', ...
             'Position', [0 0 8 9], 'Visible', 'off');
tl  = tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for p = 1:6
    ax = nexttile(tl);
    if size(imgs{p}, 3) == 3
        imshow(imgs{p}, 'Parent', ax);
    else
        imshow(imgs{p}, [], 'Parent', ax);
    end
    title(ax, labels{p}, 'FontSize', 9, 'FontWeight', 'normal');
end

% =========================================================================
% SAVE
% =========================================================================
outDir = fileparts(outFile);
if ~isempty(outDir) && ~isfolder(outDir)
    mkdir(outDir);
end
exportgraphics(fig, outFile, 'Resolution', 300);
close(fig);
fprintf('Saved: %s\n', outFile);

% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function gray = readGrayFrame(path)
img = imread(path);
if size(img, 3) == 3
    gray = rgb2gray(img);
else
    gray = img;
end
end

function [rb, ri, bgM, fsu, fgD] = initState(H, W, cfg)
rb  = zeros(H, W, cfg.ringBufLen, 'uint8');
ri  = 1;
bgM = int16(zeros(H, W));
fsu = 0;
fgD = vision.ForegroundDetector('NumGaussians', 3, ...
    'NumTrainingFrames', 60, 'LearningRate', 0.005);
end

function [rb, ri, bgM, fsu] = advanceBuffer(rb, ri, bgM, fsu, gray, cfg)
% Insert gray into ring buffer and conditionally refresh the median background.
% Mirrors the updateRingBuf + applyBackground logic from the live pipeline.
rb(:,:,ri) = gray;
ri  = mod(ri, cfg.ringBufLen) + 1;
nF  = min(cfg.medianBufLen, cfg.ringBufLen);
fsu = fsu + 1;
if fsu >= cfg.bgUpdateInterval || all(bgM(:) == 0)
    bgM = int16(median(rb(:,:,1:cfg.bgMedianStride:nF), 3));
    fsu = 0;
end
end
