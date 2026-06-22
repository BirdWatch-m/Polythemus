function [mRef, mTgt, perFrame] = poolPairMatches(framesRef, framesTgt, p)
% POOLPAIRMATCHES Pools feature matches across synchronized frame pairs.


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

    mRef = [mRef; v1(pr(:,1)).Location];
    mTgt = [mTgt; v2(pr(:,2)).Location];
    perFrame(k) = size(pr, 1);
end

end

function g = toGray(im)
if size(im, 3) == 3, g = rgb2gray(im); else, g = im; end
end
