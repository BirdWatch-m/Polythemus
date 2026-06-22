function [R_rel, t_rel, info] = relativePoseFromMatches(mRef, mTgt, intrRef, intrTgt, p)
% RELATIVEPOSEFROMMATCHES Estimates relative camera pose from pooled matches.


info = struct('nPooled', size(mRef,1), 'nInliers', 0, 'inlierFrac', 0, ...
              'epiMedian', NaN, 'F', [], 'constrained', false);

if size(mRef, 1) < p.minPooledInliers
    error('relativePoseFromMatches:tooFew', ...
          ['Only %d pooled matches (need >= %d). Capture longer, aim at a more ' ...
           'textured scene region, or improve lighting.'], ...
          size(mRef,1), p.minPooledInliers);
end

[F, inl] = estimateFundamentalMatrix(mRef, mTgt, 'Method', 'RANSAC', ...
    'NumTrials',         p.ransacNumTrials, ...
    'DistanceThreshold', p.ransacDistance, ...
    'Confidence',        p.ransacConfidence);

info.nInliers   = sum(inl);
info.inlierFrac = mean(inl);
info.F          = F;

in1 = mRef(inl, :);
in2 = mTgt(inl, :);

maxN = 20000;
if isfield(p, 'maxPoseMatches') && ~isempty(p.maxPoseMatches), maxN = p.maxPoseMatches; end
if size(in1, 1) > maxN
    sel  = unique(round(linspace(1, size(in1,1), maxN)));
    pin1 = in1(sel, :); pin2 = in2(sel, :);
else
    pin1 = in1; pin2 = in2;
end

[relOri, relLoc] = relativeCameraPose(F, intrRef, intrTgt, pin1, pin2);
R_rel = relOri;
t_rel = -R_rel * relLoc.';
t_rel = t_rel / norm(t_rel);

zeroLevel = isfield(p, 'fixCam2Level')    && p.fixCam2Level;
zeroDepth = isfield(p, 'fixCam2Coplanar') && p.fixCam2Coplanar;
if zeroLevel || zeroDepth
    [R_rel, t_rel] = constrainCam2Centre(R_rel, t_rel, zeroLevel, zeroDepth);
    info.constrained = true;
end

info.epiMedian = median(symEpiDist(in1, in2, F));

end

function d = symEpiDist(P1, P2, F)
n  = size(P1, 1);
p1 = [P1 ones(n,1)].';
p2 = [P2 ones(n,1)].';
l2 = F * p1;  l1 = F.' * p2;
num = abs(sum(p2 .* l2, 1));
d   = 0.5 * (num ./ hypot(l1(1,:), l1(2,:)) + num ./ hypot(l2(1,:), l2(2,:)));
d   = d(:);
end
