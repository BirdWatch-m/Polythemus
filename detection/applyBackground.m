function [mask, bgMedian] = applyBackground(frameGray, bgMedian, ringBuf_i, fgDetector, cfg)
% APPLYBACKGROUND  Compute foreground mask using temporal median AND GMM.
%
%   [mask, bgMedian] = applyBackground(frameGray, bgMedian, ringBuf_i, fgDetector, cfg)
%
%   Two models run in parallel and are ANDed together:
%     Median model — per-pixel median of the last cfg.medianBufLen frames.
%                    Robust to slow cloud drift and gradual lighting changes.
%     GMM model    — vision.ForegroundDetector adaptive per-pixel model.
%                    Handles faster local variation and updates each frame.
%
%   A pixel is flagged as foreground only if BOTH models agree.
%   This reduces false positives from clouds and exposure shifts.
%
%   INPUTS
%     frameGray  — H x W uint8 grayscale current frame
%     bgMedian   — H x W double current median background estimate
%     ringBuf_i  — H x W x ringBufLen uint8 ring buffer for this camera
%     fgDetector — vision.ForegroundDetector object for this camera
%     cfg        — struct from buildConfig()
%
%   OUTPUTS
%     mask      — H x W logical foreground mask
%     bgMedian  — H x W double updated median background
%
%   See also: preprocessFrame, gateBlobs

% Update median background from the filled portion of the ring buffer every
% 15 frames.
% median() along dim 3 gives per-pixel median across stored frames.
nFrames  = min(cfg.medianBufLen, cfg.ringBufLen);
persistent framesSinceUpdate;
if isempty(framesSinceUpdate), framesSinceUpdate = 0; end

framesSinceUpdate = framesSinceUpdate + 1;
if framesSinceUpdate >= 15 || all(bgMedian(:) == 0)
    bgMedian = double(median(ringBuf_i(:,:,1:nFrames), 3));
    framesSinceUpdate = 0;
end

% Median foreground: absolute difference from median background, thresholded.
diffImg    = abs(double(frameGray) - bgMedian);
mask_median = diffImg > 35;   % threshold in intensity units (0-255); tune if needed

% GMM foreground.
mask_gmm = step(fgDetector, frameGray);

% Both models must agree.
mask = mask_median & mask_gmm;

end
