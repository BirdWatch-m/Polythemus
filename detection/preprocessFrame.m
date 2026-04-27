function [blobs, bgMedian] = preprocessFrame(frame, bgMedian, ringBuf_i, fgDetector, skyMask, cfg)
% PREPROCESSFRAME  Full detection pipeline for one camera, one frame.
%
%   [blobs, bgMedian] = preprocessFrame(frame, bgMedian, ringBuf_i, fgDetector, skyMask, cfg)
%
%   Runs in sequence: grayscale conversion, background subtraction,
%   sky masking, morphological cleanup, blob extraction and gating.
%
%   INPUTS
%     frame      — H x W x 3 uint8 colour frame
%     bgMedian   — H x W double current median background for this camera
%     ringBuf_i  — H x W x ringBufLen uint8 ring buffer for this camera
%     fgDetector — vision.ForegroundDetector for this camera
%     skyMask    — H x W logical sky region mask for this camera
%     cfg        — struct from buildConfig()
%
%   OUTPUTS
%     blobs     — struct array of gated detections (see gateBlobs)
%     bgMedian  — updated median background
%
%   See also: detectBlobs, applyBackground, gateBlobs

frameGray = rgb2gray(frame);

[mask, bgMedian] = applyBackground(frameGray, bgMedian, ringBuf_i, fgDetector, cfg);

blobs = gateBlobs(mask, skyMask, cfg);

end
