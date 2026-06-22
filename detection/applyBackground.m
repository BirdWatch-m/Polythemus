function [mask, bgMedian, framesSinceUpdate] = applyBackground(frameGray, bgMedian, ringBuf_i, framesSinceUpdate, fgDetector, cfg)
% APPLYBACKGROUND Builds a foreground mask from median and GMM models.


nFrames  = min(cfg.medianBufLen, cfg.ringBufLen);

framesSinceUpdate = framesSinceUpdate + 1;
if framesSinceUpdate >= cfg.bgUpdateInterval || all(bgMedian(:) == 0)
    bgMedian = int16(median(ringBuf_i(:,:,1:cfg.bgMedianStride:nFrames), 3));
    framesSinceUpdate = 0;
end

diffImg    = abs(int16(frameGray) - bgMedian);
mask_median = diffImg > cfg.medianFgThreshold;

if cfg.useGMM
    mask_gmm = step(fgDetector, frameGray);
    mask     = mask_median & mask_gmm;
else
    mask     = mask_median;
end

end
