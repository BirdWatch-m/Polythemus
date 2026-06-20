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
%   CONSTRAINED MODE (p.fixCam2Coplanar = true)
%     The target camera's forward (depth) offset along the reference optical axis
%     is near-unobservable from a distant scene — it carries almost no epipolar
%     signal, so it drifts run-to-run. With fixCam2Coplanar the recovered rotation
%     and the lateral baseline direction are KEPT, but the forward component of the
%     target's optical centre is forced to zero (coplanar with the reference).
%     Correct for a level side-by-side rig; do not use if the cameras are
%     genuinely staggered in depth.
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

[relOri, relLoc] = relativeCameraPose(F, intrRef, intrTgt, in1, in2);
R_rel = relOri;                      % premultiply convention — do NOT transpose
t_rel = -R_rel * relLoc.';
t_rel = t_rel / norm(t_rel);         % unit; caller applies knownBaseline

if isfield(p, 'fixCam2Coplanar') && p.fixCam2Coplanar
    % Drop the unobservable forward (depth) component of the target's centre.
    C2 = -R_rel.' * t_rel;           % target optical centre direction (ref frame)
    C2(3) = 0;                        % zero forward offset -> coplanar with reference
    if norm(C2) < eps
        error('relativePoseFromMatches:degenerateConstraint', ...
              'Lateral baseline vanished under the coplanar constraint.');
    end
    C2 = C2 / norm(C2);
    t_rel = -R_rel * C2;             % stays unit (R orthonormal)
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
