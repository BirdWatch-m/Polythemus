function [mRef, mTgt, perFrame] = poolPairMatches(framesRef, framesTgt, p)
% POOLPAIRMATCHES  Concatenate SURF correspondences across synchronized frame pairs.
%
%   [mRef, mTgt, perFrame] = poolPairMatches(framesRef, framesTgt, p)
%
%   Detects and matches SURF features in each synchronized (cam-ref, cam-tgt)
%   frame pair and concatenates all matches into one pooled set. Pooling over
%   many frames is what makes the true scene geometry the dominant consensus:
%   genuine correspondences recur consistently across frames, while spurious
%   matches from repetitive structure (building façades) do not, so they are
%   diluted. A single pair is multi-modal under RANSAC; the pool is not.
%
%   Deterministic — no RANSAC here. The robust fit happens in
%   relativePoseFromMatches.
%
%   INPUTS
%     framesRef, framesTgt — 1xK cell arrays of frames (RGB or grayscale),
%                            element k from each camera captured simultaneously
%     p                    — cfg.calExtrinsics (uses surfMetricThresh,
%                            matchThreshold, maxRatio)
%
%   OUTPUTS
%     mRef, mTgt — pooled matched pixel locations, [M x 2] each (row m in mRef
%                  corresponds to row m in mTgt)
%     perFrame   — 1xK count of matches contributed by each frame pair
%
%   See also: relativePoseFromMatches, calibrateExtrinsics, epipolarOnRecording

K = numel(framesRef);
mRef = zeros(0, 2);
mTgt = zeros(0, 2);
perFrame = zeros(1, K);

for k = 1:K
    g1 = toGray(framesRef{k});
    g2 = toGray(framesTgt{k});

    f1 = detectSURFFeatures(g1, 'MetricThreshold', p.surfMetricThresh);
    f2 = detectSURFFeatures(g2, 'MetricThreshold', p.surfMetricThresh);
    if f1.Count < 8 || f2.Count < 8
        continue;
    end

    [d1, v1] = extractFeatures(g1, f1);
    [d2, v2] = extractFeatures(g2, f2);
    pr = matchFeatures(d1, d2, 'MatchThreshold', p.matchThreshold, 'MaxRatio', p.maxRatio);
    if isempty(pr)
        continue;
    end

    mRef = [mRef; v1(pr(:,1)).Location]; %#ok<AGROW>
    mTgt = [mTgt; v2(pr(:,2)).Location]; %#ok<AGROW>
    perFrame(k) = size(pr, 1);
end

end


function g = toGray(im)
if size(im, 3) == 3, g = rgb2gray(im); else, g = im; end
end
