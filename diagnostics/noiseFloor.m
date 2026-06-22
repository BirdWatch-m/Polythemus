% NOISEFLOOR  Find the minimum safe medianFgThreshold for this recording.
%
%   Warms up the background model on the first warmupFrames frames, then
%   analyses analyseRange (5 s of quiet sky before birds appear). Outputs:
%     - Percentiles of diffImg pixel values in the sky region (noise floor)
%     - Blob count + foreground pixel count at each candidate threshold,
%       for median-only AND median+GMM detection
%
%   This tells you exactly where to set medianFgThreshold: just above the
%   99.9th percentile of diffImg in a bird-free window.
%
%   See also: buildConfig, applyBackground, gateBlobs

clc; close all; clear;
addpath(genpath(fileparts(mfilename('fullpath'))));

% =========================================================================
% USER INPUTS
% =========================================================================
recordingDir = 'output/recordings/20260621_205602';
camIdx       = 1;           % camera to analyse (1 or 2; both sky should agree)
warmupFrames = 90;          % frames used to initialise background model (~3 s)
analyseRange = 91:240;      % 5 s of bird-free sky to characterise
thresholds   = [5 8 10 12 15 20 25 30 35];

% =========================================================================
% SETUP
% =========================================================================
loaded = load(fullfile(recordingDir, 'session.mat'));
cfg    = buildConfig();
cfg.N          = loaded.session.cfg.N;
cfg.resolution = loaded.session.cfg.resolution;
cfg.fps        = loaded.session.cfg.fps;
cfg.ringBufLen = loaded.session.cfg.ringBufLen;
H = cfg.resolution(2);  W = cfg.resolution(1);

if isfile(cfg.skyMaskFile)
    sm      = load(cfg.skyMaskFile);
    skyMask = sm.skyMasks{camIdx};
    fprintf('Sky mask: %d / %d px (%.0f%% of frame).\n', ...
            sum(skyMask(:)), H*W, 100*mean(skyMask(:)));
else
    skyMask = true(H, W);
    fprintf('No sky mask found — using full frame.\n');
end

readFrame = @(k) im2gray(imread(fullfile(recordingDir, ...
    sprintf('cam%d', camIdx), sprintf('frame_%06d.tif', k))));

ringBuf  = zeros(H, W, cfg.ringBufLen, 'uint8');
ringIdx  = 1;
bgMedian = int16(zeros(H, W));
bgFSU    = 0;
fgDet    = vision.ForegroundDetector('NumGaussians', 3, ...
               'NumTrainingFrames', 60, 'LearningRate', 0.005);

% =========================================================================
% WARMUP
% =========================================================================
fprintf('Warming up background model (%d frames)...', warmupFrames);
for k = 1:warmupFrames
    f = readFrame(k);
    [ringBuf, ringIdx, bgMedian, bgFSU] = feedFrame(f, ringBuf, ringIdx, bgMedian, bgFSU, cfg);
    step(fgDet, f);
end
fprintf(' done.\n\n');

% =========================================================================
% COLLECT diffImg + GMM MASKS FOR ANALYSIS WINDOW
% =========================================================================
nA       = numel(analyseRange);
diffImgs = zeros(H, W, nA, 'int16');   % ~276 MB for 720p x 150 frames
gmmMasks = false(H, W, nA);

fprintf('Collecting %d analysis frames...', nA);
for ki = 1:nA
    k = analyseRange(ki);
    f = readFrame(k);
    [ringBuf, ringIdx, bgMedian, bgFSU] = feedFrame(f, ringBuf, ringIdx, bgMedian, bgFSU, cfg);
    diffImgs(:,:,ki) = abs(int16(f) - bgMedian);
    gmmMasks(:,:,ki) = step(fgDet, f);
end
fprintf(' done.\n\n');

% =========================================================================
% NOISE FLOOR PERCENTILES  (median model, sky pixels only)
% =========================================================================
skyMask3D  = repmat(skyMask, 1, 1, nA);
skyPixels  = double(diffImgs(skyMask3D));
pcts       = prctile(skyPixels, [90 95 99 99.5 99.9 99.99]);

fprintf('diffImg percentiles — sky pixels, %d frames, cam %d:\n', nA, camIdx);
fprintf('  90%%    : %5.1f\n', pcts(1));
fprintf('  95%%    : %5.1f\n', pcts(2));
fprintf('  99%%    : %5.1f\n', pcts(3));
fprintf('  99.5%%  : %5.1f\n', pcts(4));
fprintf('  99.9%%  : %5.1f\n', pcts(5));
fprintf('  99.99%% : %5.1f\n', pcts(6));
fprintf('\n  --> minimum safe threshold (above 99.9%% noise): ~%.0f\n\n', ceil(pcts(5)));

% =========================================================================
% THRESHOLD SWEEP  — blob counts + foreground pixel counts
% =========================================================================
fprintf('%-10s  %-18s  %-18s  %-14s  %-14s\n', ...
    'Threshold', 'Median blobs/fr', 'Med+GMM blobs/fr', ...
    'FG px/fr (med)', 'FG px/fr (m+g)');
fprintf('%s\n', repmat('-', 1, 78));

for thr = thresholds
    cfg.medianFgThreshold = thr;
    bMed = zeros(1, nA);  bMG = zeros(1, nA);
    pMed = zeros(1, nA);  pMG = zeros(1, nA);
    for ki = 1:nA
        mMed = (diffImgs(:,:,ki) > thr) & skyMask;
        mMG  = mMed & gmmMasks(:,:,ki);
        pMed(ki) = sum(mMed(:));
        pMG(ki)  = sum(mMG(:));
        bMed(ki) = numel(gateBlobs(mMed, skyMask, cfg));
        bMG(ki)  = numel(gateBlobs(mMG,  skyMask, cfg));
    end
    fprintf('%-10d  %-18s  %-18s  %-14.0f  %-14.0f\n', thr, ...
        sprintf('%.1f (max %d)', mean(bMed), max(bMed)), ...
        sprintf('%.1f (max %d)', mean(bMG),  max(bMG)), ...
        mean(pMed), mean(pMG));
end
fprintf('%s\n', repmat('-', 1, 78));

% =========================================================================
% LOCAL HELPERS
% =========================================================================
function [ringBuf, ringIdx, bgMedian, bgFSU] = feedFrame(f, ringBuf, ringIdx, bgMedian, bgFSU, cfg)
ringBuf(:,:,ringIdx) = f;
ringIdx = mod(ringIdx, cfg.ringBufLen) + 1;
bgFSU   = bgFSU + 1;
if bgFSU >= cfg.bgUpdateInterval || all(bgMedian(:) == 0)
    nF       = min(cfg.medianBufLen, cfg.ringBufLen);
    bgMedian = int16(median(ringBuf(:,:,1:cfg.bgMedianStride:nF), 3));
    bgFSU    = 0;
end
end
