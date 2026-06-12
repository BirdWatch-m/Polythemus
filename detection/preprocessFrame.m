function [blobs, bgMedian, framesSinceUpdate] = preprocessFrame(frameGray, bgMedian, ringBuf_i, framesSinceUpdate, fgDetector, skyMask, cfg)
% PREPROCESSFRAME  Full detection pipeline for one camera, one frame.
%
%   [blobs, bgMedian, framesSinceUpdate] = preprocessFrame(frameGray, ...
%       bgMedian, ringBuf_i, framesSinceUpdate, fgDetector, skyMask, cfg)
%
%   Runs in sequence: background subtraction, sky masking, morphological
%   cleanup, blob extraction and gating. The grayscale conversion happens
%   upstream (updateRingBuf) so it is done once per frame, not twice.
%
%   INPUTS
%     frameGray         — H x W uint8 grayscale current frame
%     bgMedian          — H x W current median background for this camera
%     ringBuf_i         — H x W x ringBufLen uint8 ring buffer for this camera
%     framesSinceUpdate — this camera's median-refresh counter (see applyBackground)
%     fgDetector        — vision.ForegroundDetector for this camera
%     skyMask           — H x W logical sky region mask for this camera
%     cfg               — struct from buildConfig()
%
%   OUTPUTS
%     blobs             — struct array of gated detections (see gateBlobs)
%     bgMedian          — updated median background
%     framesSinceUpdate — updated median-refresh counter
%
%   See also: detectBlobs, applyBackground, gateBlobs

[mask, bgMedian, framesSinceUpdate] = applyBackground( ...
    frameGray, bgMedian, ringBuf_i, framesSinceUpdate, fgDetector, cfg);

blobs = gateBlobs(mask, skyMask, cfg);

end
