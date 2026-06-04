function [mask, bgMedian, framesSinceUpdate] = applyBackground(frameGray, bgMedian, ringBuf_i, framesSinceUpdate, fgDetector, cfg)
% APPLYBACKGROUND  Compute foreground mask using temporal median AND GMM.
%
%   [mask, bgMedian, framesSinceUpdate] = applyBackground(frameGray, ...
%       bgMedian, ringBuf_i, framesSinceUpdate, fgDetector, cfg)
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
%   The median is recomputed only every cfg.bgUpdateInterval frames. The
%   update counter is passed in and returned (rather than held in a
%   persistent) so each camera keeps its own counter — a single persistent
%   would be shared across all cameras and only camera 1 would ever refresh.
%
%   INPUTS
%     frameGray         — H x W uint8 grayscale current frame
%     bgMedian          — H x W double current median background estimate
%     ringBuf_i         — H x W x ringBufLen uint8 ring buffer for this camera
%     framesSinceUpdate — frames elapsed since this camera's last median refresh
%     fgDetector        — vision.ForegroundDetector object for this camera
%     cfg               — struct from buildConfig()
%
%   OUTPUTS
%     mask              — H x W logical foreground mask
%     bgMedian          — H x W double updated median background
%     framesSinceUpdate — updated counter (0 if the median was refreshed)
%
%   See also: preprocessFrame, gateBlobs

% Update median background from the filled portion of the ring buffer every
% cfg.bgUpdateInterval frames. The buffer is subsampled by cfg.bgMedianStride
% to keep the median cheap — on slowly-drifting sky a sparse sample is fine.
% median() along dim 3 gives per-pixel median across the sampled frames.
nFrames  = min(cfg.medianBufLen, cfg.ringBufLen);

framesSinceUpdate = framesSinceUpdate + 1;
if framesSinceUpdate >= cfg.bgUpdateInterval || all(bgMedian(:) == 0)
    bgMedian = double(median(ringBuf_i(:,:,1:cfg.bgMedianStride:nFrames), 3));
    framesSinceUpdate = 0;
end

% Median foreground: absolute difference from median background, thresholded.
diffImg    = abs(double(frameGray) - bgMedian);
mask_median = diffImg > cfg.medianFgThreshold;   % intensity units (0-255)

% GMM foreground.
mask_gmm = step(fgDetector, frameGray);

% Both models must agree.
mask = mask_median & mask_gmm;

end
