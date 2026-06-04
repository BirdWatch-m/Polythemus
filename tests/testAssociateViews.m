function testAssociateViews()
% TESTASSOCIATEVIEWS  Synthetic unit test for associateViews (no hardware).
%
%   Builds a known 2-camera stereo geometry, projects 3 world points into both
%   views (these must associate into pairs), adds one unmatched blob per camera
%   (these must stay singletons), and checks associateViews groups them
%   correctly. Self-consistent: the same convention builds F and projects the
%   points, so it does not depend on the real extrinsic convention (BUG-5).
%
%   Run from the project root:
%       addpath(genpath(pwd)); testAssociateViews

% --- Synthetic 2-camera rig (720p-ish intrinsics, horizontal baseline) ---
K  = [900 0 640; 0 900 360; 0 0 1];
R1 = eye(3);  t1 = [0; 0; 0];
R2 = eye(3);  t2 = [0.5; 0; 0];          % 0.5 m baseline, parallel axes

% Fundamental matrix F{1,2}, same convention as initSystem.buildFundamentalMatrices.
R_rel = R2 * R1.';
t_rel = t2 - R_rel * t1;
Tx    = [0 -t_rel(3) t_rel(2); t_rel(3) 0 -t_rel(1); -t_rel(2) t_rel(1) 0];
E     = Tx * R_rel;
F12   = inv(K).' * E * inv(K);

cal.F = cell(2, 2);
cal.F{1,2} = F12;
cal.F{2,1} = F12.';

cfg.N = 2;
cfg.epiThreshold = 2.0;

% --- 3 world points (Z>0) projected into both cameras: must pair up ---
P = [ -1.0  0.5   8 ;
       0.3 -0.4  12 ;
       1.0  0.8  10 ].';            % 3x3, columns = points
nPts = size(P, 2);

blobs = {struct('centroid', {}), struct('centroid', {})};
for p = 1:nPts
    x1 = project(K, R1, t1, P(:,p));
    x2 = project(K, R2, t2, P(:,p));
    blobs{1}(end+1).centroid = x1(:).';
    blobs{2}(end+1).centroid = x2(:).';
end

% --- One unmatched blob per camera, at frame-edge rows far from the points.
% (Rectified stereo -> horizontal epipolar lines, so distinct rows never match.)
blobs{1}(end+1).centroid = [320,  10];
blobs{2}(end+1).centroid = [960, 700];

% --- Run ---
groups = associateViews(blobs, cal, cfg);

% --- Checks ---
nPair   = sum([groups.nViews] == 2);
nSingle = sum([groups.nViews] == 1);

assert(nPair == nPts,  'Expected %d paired groups, got %d.',   nPts, nPair);
assert(nSingle == 2,   'Expected 2 singleton groups, got %d.', nSingle);

% Every paired group must satisfy the epipolar constraint tightly.
for g = find([groups.nViews] == 2)
    a = groups(g).blobIdx(1);  b = groups(g).blobIdx(2);
    x1   = [blobs{1}(a).centroid, 1].';
    x2   = [blobs{2}(b).centroid, 1].';
    line = F12 * x1;
    dist = abs(x2.' * line) / hypot(line(1), line(2));   % point-to-line dist (px)
    assert(dist < 1e-3, 'Paired group %d epipolar distance too large: %.4g px.', g, dist);
end

fprintf('testAssociateViews PASSED: %d paired, %d singleton groups.\n', nPair, nSingle);

end


function x = project(K, R, t, X)
p = K * (R * X + t);
x = p(1:2) / p(3);
end
