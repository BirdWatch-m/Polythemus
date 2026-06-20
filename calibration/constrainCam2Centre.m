function [R, t] = constrainCam2Centre(R, t, zeroLevel, zeroDepth)
% CONSTRAINCAM2CENTRE  Project the target camera's optical centre onto a subspace.
%
%   [R, t] = constrainCam2Centre(R, t, zeroLevel, zeroDepth)
%
%   Zeros the vertical (Y) and/or forward (Z) component of the target camera's
%   optical centre in the reference frame, keeping the rotation unchanged and the
%   baseline magnitude |t| preserved. Used to remove degrees of freedom that are
%   either unobservable (forward/depth, at low parallax) or known to be zero
%   (vertical, for a level side-by-side rig). Method-agnostic: works on a unit
%   pose (SURF) or a metric pose (checkerboard) because |t| is preserved, so the
%   zeroed magnitude is redistributed into the lateral baseline.
%
%   Reference frame axes (camera 1): X right, Y down, Z forward into the scene.
%
%   INPUTS
%     R, t       — target pose relative to reference (premultiply: X_tgt = R*X_ref + t)
%     zeroLevel  — logical; force the vertical (Y) centre offset to 0
%     zeroDepth  — logical; force the forward (Z) centre offset to 0
%
%   OUTPUTS
%     R — unchanged
%     t — adjusted so the constrained axes of the target centre are 0, |t| kept
%
%   See also: relativePoseFromMatches, calibrateExtrinsics,
%             calibrateExtrinsicsCheckerboard

if ~zeroLevel && ~zeroDepth
    return;
end

C   = -R.' * t;            % target optical centre in reference frame
mag = norm(C);
if zeroLevel, C(2) = 0; end
if zeroDepth, C(3) = 0; end

if norm(C) < eps
    error('constrainCam2Centre:degenerate', ...
          'Baseline vanished under the requested constraint (no lateral component).');
end

C = C / norm(C) * mag;     % restore the original baseline magnitude
t = -R * C;
end
