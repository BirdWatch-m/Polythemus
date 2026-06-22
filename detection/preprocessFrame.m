function [blobs, bgMedian, framesSinceUpdate, counts] = preprocessFrame(frameGray, bgMedian, ringBuf_i, framesSinceUpdate, fgDetector, skyMask, cfg)
% PREPROCESSFRAME Runs detection preprocessing for one camera frame.


[mask, bgMedian, framesSinceUpdate] = applyBackground( ...
    frameGray, bgMedian, ringBuf_i, framesSinceUpdate, fgDetector, cfg);

[blobs, counts] = gateBlobs(mask, skyMask, cfg);

end
