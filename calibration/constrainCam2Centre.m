function [R, t] = constrainCam2Centre(R, t, zeroLevel, zeroDepth)
% CONSTRAINCAM2CENTRE Constrains a relative camera centre while preserving baseline length.


if ~zeroLevel && ~zeroDepth
    return;
end

C   = -R.' * t;
mag = norm(C);
if zeroLevel, C(2) = 0; end
if zeroDepth, C(3) = 0; end

if norm(C) < eps
    error('constrainCam2Centre:degenerate', ...
          'Baseline vanished under the requested constraint (no lateral component).');
end

C = C / norm(C) * mag;
t = -R * C;
end
