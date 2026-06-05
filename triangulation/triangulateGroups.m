function points = triangulateGroups(groups, calibration, cfg)
% TRIANGULATEGROUPS  Recover 3D positions from associated multi-view blobs.
%
%   points = triangulateGroups(groups, calibration, cfg)
%
%   For each association group seen by >= 2 cameras (from associateViews), the
%   observed centroids are undistorted, triangulated to a single world point by
%   the linear DLT (direct linear transform) method, and then gated by mean
%   reprojection error. A group whose error exceeds cfg.reprThreshold is kept
%   but flagged invalid (a likely bad association that slipped the epipolar
%   gate). Groups with < 2 views are skipped (not triangulatable).
%
%   DLT: each view gives two linear rows from x cross (P*X) = 0; stacking all
%   views yields A*X = 0, solved for the homogeneous world point X as the right
%   singular vector of A for the smallest singular value (SVD null space).
%
%   CONVENTION (relevant to BUG-5)
%     Projection matrices are built as P = K * [R | t], i.e. R, t are taken as
%     WORLD-to-CAMERA (X_cam = R * X_world + t), consistent with how initSystem
%     builds the fundamental matrices. If BUG-5 validation shows the stored
%     R, t are camera-to-world, build P = K * [R' | -R'*t] instead. The DLT
%     itself is correct either way; only which convention is right is open.
%
%   LIMITATION
%     A mismatch that lies ALONG the epipolar line is geometrically consistent —
%     the two rays still intersect, just at the wrong depth — so it passes both
%     the epipolar gate (associateViews) and this reprojection gate. Two views
%     cannot detect such an error; a 3rd camera (a differently oriented epipolar
%     constraint) or temporal tracking is needed to disambiguate it.
%
%   INPUTS
%     groups      — struct array from associateViews (.camIds, .points [Nx2])
%     calibration — struct with .intrinsics{i} (cameraParameters/cameraIntrinsics),
%                   .R{i} (3x3), .t{i} (3x1)
%     cfg         — struct from buildConfig (uses cfg.N, cfg.reprThreshold)
%
%   OUTPUT
%     points — 1xP struct array, one per triangulated group, with fields:
%       .position  — [1x3] world coordinates (metres)
%       .reprojErr — mean reprojection error over observing cameras (pixels)
%       .camIds    — cameras that observed it
%       .groupIdx  — index into the input groups
%       .valid     — reprojErr <= cfg.reprThreshold
%
%   See also: associateViews, initSystem, validateCalibration

N = cfg.N;

% Per-camera projection matrices P = K * [R | t].
P = cell(1, N);
for i = 1:N
    K    = calibration.intrinsics{i}.IntrinsicMatrix.';   % standard K (column convention)
    P{i} = K * [calibration.R{i}, calibration.t{i}(:)];
end

points = repmat(emptyPoint(), 1, 0);

for g = 1:numel(groups)
    cams = groups(g).camIds;
    if numel(cams) < 2
        continue;   % need at least two views to triangulate
    end

    % Undistort each observing camera's centroid, collect its projection matrix.
    obs = zeros(numel(cams), 2);
    Ps  = cell(1, numel(cams));
    for c = 1:numel(cams)
        i        = cams(c);
        obs(c,:) = undistortPoints(groups(g).points(i,:), calibration.intrinsics{i});
        Ps{c}    = P{i};
    end

    [X, reprojErr] = triangulateDLT(obs, Ps);

    p           = emptyPoint();
    p.position  = X(:).';
    p.reprojErr = reprojErr;
    p.camIds    = cams;
    p.groupIdx  = g;
    p.valid     = reprojErr <= cfg.reprThreshold;
    points(end+1) = p; %#ok<AGROW>
end

end


% =========================================================================
% LOCAL HELPERS
% =========================================================================

function p = emptyPoint()
p = struct('position', [NaN NaN NaN], 'reprojErr', NaN, ...
           'camIds', [], 'groupIdx', 0, 'valid', false);
end


function [X, reprojErr] = triangulateDLT(pts, Ps)
% Linear DLT triangulation from K views.
%   pts — K x 2 undistorted image points (one row per view)
%   Ps  — 1 x K cell of 3x4 projection matrices
%   X   — 3x1 world point; reprojErr — mean reprojection error (pixels)
K = numel(Ps);

A = zeros(2*K, 4);
for c = 1:K
    Pc = Ps{c};
    u  = pts(c,1); v = pts(c,2);
    A(2*c-1, :) = u * Pc(3,:) - Pc(1,:);
    A(2*c,   :) = v * Pc(3,:) - Pc(2,:);
end

[~, ~, V] = svd(A, 0);
Xh = V(:, end);
X  = Xh(1:3) / Xh(4);

errs = zeros(K, 1);
for c = 1:K
    x       = Ps{c} * [X; 1];
    x       = x(1:2) / x(3);
    errs(c) = hypot(x(1) - pts(c,1), x(2) - pts(c,2));
end
reprojErr = mean(errs);
end
