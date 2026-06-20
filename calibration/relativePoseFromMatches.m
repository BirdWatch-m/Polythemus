function [R_rel, t_rel, info] = relativePoseFromMatches(mRef, mTgt, intrRef, intrTgt, p)
% RELATIVEPOSEFROMMATCHES  Robust relative camera pose from pooled correspondences.
%
%   [R_rel, t_rel, info] = relativePoseFromMatches(mRef, mTgt, intrRef, intrTgt, p)
%
%   Fits one fundamental matrix to the pooled matches with RANSAC, then recovers
%   the relative pose of the target camera w.r.t. the reference camera.
%
%   CONVENTION
%     World-to-camera premultiply (X_cam = R*X_world + t), matching the rest of
%     the pipeline. relativeCameraPose's orientation is already that R — it is
%     NOT transposed (verified in tests/testExtrinsicConventions.m). t_rel is a
%     UNIT vector; the caller applies metric scale (knownBaseline).
%
%   INPUTS
%     mRef, mTgt       — pooled matched pixel locations [M x 2] each
%     intrRef, intrTgt — cameraParameters / cameraIntrinsics for each camera
%     p                — cfg.calExtrinsics (ransacNumTrials, ransacDistance,
%                        ransacConfidence, minPooledInliers, fixCam2Coplanar)
%
%   CONSTRAINED MODE (p.fixCam2Coplanar and/or p.fixCam2Level)
%     The recovered rotation is always KEPT; only the target's optical centre is
%     projected (via constrainCam2Centre). fixCam2Coplanar zeros the forward
%     (depth) offset — near-unobservable at low parallax, so it otherwise drifts.
%     fixCam2Level zeros the vertical offset — observable, but asserted zero for a
%     level rig. Both set => cam2 is a pure lateral baseline [B,0,0]. Correct for a
%     level side-by-side rig; do not use if the cameras are genuinely staggered.
%
%   OUTPUTS
%     R_rel — 3x3 rotation, target relative to reference (premultiply)
%     t_rel — 3x1 UNIT translation direction, target relative to reference
%     info  — struct: nPooled, nInliers, inlierFrac, epiMedian (px), F, constrained
%
%   ERRORS
%     relativePoseFromMatches:tooFew  — fewer than p.minPooledInliers matches
%
%   See also: poolPairMatches, calibrateExtrinsics, buildFundamentalMatrices

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

% Cap the inlier set fed to the pose decomposition. relativeCameraPose is O(n^2)
% in memory, so tens of thousands of inliers OOM; a few thousand well-spread
% points already determine the pose. F above is fit on ALL pooled matches.
maxN = 20000;
if isfield(p, 'maxPoseMatches') && ~isempty(p.maxPoseMatches), maxN = p.maxPoseMatches; end
if size(in1, 1) > maxN
    sel  = unique(round(linspace(1, size(in1,1), maxN)));   % deterministic, spread over frames
    pin1 = in1(sel, :); pin2 = in2(sel, :);
else
    pin1 = in1; pin2 = in2;
end

[relOri, relLoc] = relativeCameraPose(F, intrRef, intrTgt, pin1, pin2);
R_rel = relOri;                      % premultiply convention — do NOT transpose
t_rel = -R_rel * relLoc.';
t_rel = t_rel / norm(t_rel);         % unit; caller applies knownBaseline

zeroLevel = isfield(p, 'fixCam2Level')    && p.fixCam2Level;     % vertical (Y)
zeroDepth = isfield(p, 'fixCam2Coplanar') && p.fixCam2Coplanar;  % forward (Z)
if zeroLevel || zeroDepth
    [R_rel, t_rel] = constrainCam2Centre(R_rel, t_rel, zeroLevel, zeroDepth);
    info.constrained = true;
end

info.epiMedian = median(symEpiDist(in1, in2, F));

end


function d = symEpiDist(P1, P2, F)
% Symmetric point-to-epipolar-line distance (pixels), vectorised.
n  = size(P1, 1);
p1 = [P1 ones(n,1)].';
p2 = [P2 ones(n,1)].';
l2 = F * p1;  l1 = F.' * p2;
num = abs(sum(p2 .* l2, 1));
d   = 0.5 * (num ./ hypot(l1(1,:), l1(2,:)) + num ./ hypot(l2(1,:), l2(2,:)));
d   = d(:);
end
