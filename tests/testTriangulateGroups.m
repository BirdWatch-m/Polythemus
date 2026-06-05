function testTriangulateGroups()
% TESTTRIANGULATEGROUPS  Synthetic round-trip test for triangulateGroups (no hardware).
%
%   Builds a known 2-camera rig, projects 3 known world points, packages them as
%   association groups, triangulates, and checks the recovered 3D positions match
%   the originals (and pass the reprojection gate). Also checks that a group with
%   an inconsistent observation is flagged invalid by the gate. Self-consistent,
%   so it does not depend on the real extrinsic convention (BUG-5).
%
%   Run from the project root:
%       addpath(genpath(pwd)); testTriangulateGroups

% --- Synthetic 2-camera rig (zero distortion) ---
K  = [900 0 640; 0 900 360; 0 0 1];
R1 = eye(3);  t1 = [0; 0; 0];
R2 = eye(3);  t2 = [0.5; 0; 0];

cal.intrinsics = { cameraIntrinsics([900 900], [640 360], [720 1280]), ...
                   cameraIntrinsics([900 900], [640 360], [720 1280]) };
cal.R = {R1, R2};
cal.t = {t1, t2};

cfg.N = 2;
cfg.reprThreshold = 1.0;

% --- True 3D points (Z>0) ---
Ptrue = [ -1.0  0.5   8 ;
           0.3 -0.4  12 ;
           1.0  0.8  10 ].';        % 3x3, columns = points
nPts = size(Ptrue, 2);

% --- Package as association groups (as associateViews would output) ---
g0 = struct('blobIdx', [0 0], 'points', nan(2,2), 'camIds', [], 'nViews', 0);
groups = repmat(g0, 1, nPts);
for p = 1:nPts
    groups(p).points(1,:) = project(K, R1, t1, Ptrue(:,p));
    groups(p).points(2,:) = project(K, R2, t2, Ptrue(:,p));
    groups(p).blobIdx = [p p];
    groups(p).camIds  = [1 2];
    groups(p).nViews  = 2;
end

% --- Triangulate and check recovery ---
points = triangulateGroups(groups, cal, cfg);

assert(numel(points) == nPts, 'Expected %d points, got %d.', nPts, numel(points));
for p = 1:nPts
    posErr = norm(points(p).position - Ptrue(:,p).');
    assert(posErr < 1e-3, 'Point %d position error %.4g m too large.', p, posErr);
    assert(points(p).valid, 'Point %d should pass the reprojection gate.', p);
    assert(points(p).reprojErr < 1e-3, 'Point %d reproj error %.4g px too large.', p, points(p).reprojErr);
end

% --- Negative case: an OFF-epipolar-line observation must fail the gate.
% Shift cam2 VERTICALLY: epipolar lines here are horizontal, so a horizontal
% shift would stay epipolar-consistent and triangulate to a valid wrong-depth
% point (~0 reprojection error). Only an off-line shift makes the two rays skew.
bad = groups(1);
bad.points(2,:) = bad.points(2,:) + [0, 60];
badOut = triangulateGroups(bad, cal, cfg);
assert(~badOut(1).valid, 'Off-epipolar-line group should fail the reprojection gate (err=%.2f px).', ...
       badOut(1).reprojErr);

fprintf('testTriangulateGroups PASSED: %d points recovered (<1mm), gate rejects inconsistent group.\n', nPts);

end


function x = project(K, R, t, X)
p = K * (R * X + t);
x = (p(1:2) / p(3)).';
end
