function testExtrinsicConventions()
% TESTEXTRINSICCONVENTIONS  Settle the extrinsic pose-convention question numerically.
%
%   Noise-free synthetic round-trip. Builds a KNOWN two-camera rig in the
%   pipeline's own world-to-camera premultiply convention (X_cam = R*X_world + t,
%   P = K*[R|t], exactly what triangulateGroups/buildFundamentalMatrices assume),
%   projects known geometry, then runs the *exact* pose-recovery math from each
%   calibration script and checks whether it reconstructs the known rig.
%
%   Because the ground truth is generated in the pipeline convention and the
%   recovery uses the real toolbox functions (extrinsics, relativeCameraPose),
%   this isolates pure convention/algebra errors from any data problem. A path
%   that cannot recover noise-free synthetic geometry is broken in software;
%   a path that recovers it perfectly is correct, and any field failure must
%   then be in the data (intrinsics, correspondences, scene, hardware).
%
%   Sub-tests
%     1 PIPELINE     triangulateGroups round-trips a known rig (self-consistency).
%     2 CHECKERBOARD calibrateExtrinsicsCheckerboard's R12/t12 formula.
%     3 RELPOSE      calibrateExtrinsics / salvageCalibration's relativeCameraPose
%                    conversion, cross-checked against MATLAB's own
%                    cameraPoseToExtrinsics.
%
%   Run from the project root:
%       addpath(genpath(pwd)); testExtrinsicConventions
%
%   See also: triangulateGroups, calibrateExtrinsics,
%             calibrateExtrinsicsCheckerboard, salvageCalibration

warnState = warning('off', 'all');
cleaner   = onCleanup(@() warning(warnState));
rng(7);   % reproducible

% ---- Ground-truth rig (pipeline convention: X_cam = R*X_world + t) ----
K   = [900 0 640; 0 900 360; 0 0 1];
imgSize = [720 1280];
intr1 = cameraIntrinsics([900 900], [640 360], imgSize);
intr2 = cameraIntrinsics([900 900], [640 360], imgSize);

C2gt  = [3.5; 0; 0];                       % cam2 optical centre in world (cam1) frame
R2gt  = Rz(2) * Rx(3) * Ry(-8);            % compound, deliberately non-symmetric
t2gt  = -R2gt * C2gt;                       % so X_cam2 = R2gt*(X_world - C2gt)
b     = norm(C2gt);                         % true baseline

fprintf('\n========================================================\n');
fprintf(' EXTRINSIC CONVENTION ROUND-TRIP  (noise-free synthetic)\n');
fprintf('========================================================\n');
fprintf('Ground truth: baseline %.3f m, cam2 centre [%.2f %.2f %.2f]\n', ...
        b, C2gt);

nFail = 0;
nFail = nFail + subPipeline(K, intr1, intr2, R2gt, t2gt, C2gt);
nFail = nFail + subCheckerboard(K, intr1, intr2, R2gt, t2gt, b);
nFail = nFail + subRelPose(K, intr1, intr2, R2gt, t2gt, b);
nFail = nFail + subFundamental(K, intr1, intr2, R2gt, t2gt);

fprintf('\n--------------------------------------------------------\n');
if nFail == 0
    fprintf('ALL CONVENTION SUB-TESTS PASSED.\n');
else
    fprintf('%d CONVENTION SUB-TEST(S) FAILED  <-- software bug localised.\n', nFail);
end
fprintf('--------------------------------------------------------\n');
end


% =========================================================================
% SUB-TEST 1 — pipeline self-consistency
% =========================================================================
function fail = subPipeline(K, intr1, intr2, R2, t2, ~)
fprintf('\n[1] PIPELINE  triangulateGroups round-trip\n');
cal.intrinsics = {intr1, intr2};
cal.R = {eye(3), R2};
cal.t = {[0;0;0], t2};
cfg.N = 2; cfg.reprThreshold = 1.0;

Xtrue = [ -1.0  0.5  9 ;  0.3 -0.4 14 ;  1.0  0.8 11 ; -2.0 -1.0 20 ].';
nP = size(Xtrue,2);
g0 = struct('points', nan(2,2), 'camIds', [1 2], 'nViews', 2);
groups = repmat(g0, 1, nP);
for p = 1:nP
    groups(p).points(1,:) = project(K, eye(3), [0;0;0], Xtrue(:,p));
    groups(p).points(2,:) = project(K, R2,     t2,      Xtrue(:,p));
end
pts = triangulateGroups(groups, cal, cfg);
err = max(arrayfun(@(k) norm(pts(k).position - Xtrue(:,k).'), 1:nP));
fail = err > 1e-6;
report(~fail, sprintf('max 3D recovery error %.2e m', err));
end


% =========================================================================
% SUB-TEST 2 — checkerboard path (extrinsics -> R12,t12)
% =========================================================================
function fail = subCheckerboard(K, intr1, intr2, R2gt, t2gt, ~)
fprintf('\n[2] CHECKERBOARD  R12 = Rb2''*Rb1 ;  t12 = tb2'' - R12*tb1''\n');
boardSize  = [6 9];
sq         = 0.03;
wp2D       = generateCheckerboardPoints(boardSize, sq);   % Mx2
wp3D       = [wp2D, zeros(size(wp2D,1),1)];

nPose = 12;
R_all = zeros(3,3,nPose); t_all = zeros(3,nPose); ok = true(1,nPose);
for k = 1:nPose
    % Random board-to-world pose, placed in front of both cameras.
    Rbw = Rz(40*(rand-0.5)) * Rx(35*(rand-0.5)) * Ry(35*(rand-0.5));
    Cbw = [ (rand-0.5)*1.5 ; (rand-0.5)*1.0 ; 2.0 + rand*1.5 ];
    Xw  = (Rbw * wp3D.').' + Cbw.';                       % Mx3 world points

    px1 = projAll(K, eye(3), [0;0;0], Xw);
    px2 = projAll(K, R2gt,   t2gt,    Xw);

    % Reject degenerate placements (corner behind a camera).
    if any(depth(eye(3),[0;0;0],Xw) <= 0) || any(depth(R2gt,t2gt,Xw) <= 0)
        ok(k) = false; continue;
    end

    [Rb1, tb1] = extrinsics(px1, wp2D, intr1);            % old (postmultiply) convention
    [Rb2, tb2] = extrinsics(px2, wp2D, intr2);
    R12 = Rb2' * Rb1;                                     % <-- exact script formula
    t12 = tb2' - R12 * tb1';
    R_all(:,:,k) = R12; t_all(:,k) = t12;
end
R_all = R_all(:,:,ok); t_all = t_all(:,ok);

R_mean = quatAverage(R_all);
t_mean = mean(t_all, 2);

aR = geoAngle(R_mean, R2gt);
et = norm(t_mean - t2gt);
fail = (aR > 1e-3) || (et > 1e-3);
report(~fail, sprintf('R err %.2e deg, t err %.2e m  (%d/%d poses used)', ...
        aR, et, sum(ok), nPose));
end


% =========================================================================
% SUB-TEST 3 — relativeCameraPose path (the contested one)
% =========================================================================
function fail = subRelPose(K, intr1, intr2, R2gt, t2gt, b)
fprintf('\n[3] RELPOSE  relativeCameraPose -> R_rel,t_rel  (scale set to true baseline)\n');

% Noise-free correspondences from the known rig.
nPt = 120;
Xw  = [ (rand(1,nPt)-0.5)*8 ; (rand(1,nPt)-0.5)*5 ; 6 + rand(1,nPt)*22 ];
x1  = projAll(K, eye(3), [0;0;0], Xw.');
x2  = projAll(K, R2gt,   t2gt,    Xw.');

F = estimateFundamentalMatrix(x1, x2, 'Method','RANSAC', ...
        'NumTrials', 4000, 'DistanceThreshold', 1e-3, 'Confidence', 99.99);
[relOri, relLoc] = relativeCameraPose(F, intr1, intr2, x1, x2);

% --- The two candidate conventions ---
% (A) exactly what calibrateExtrinsics & salvageCalibration do:
RA = relOri';            tA = -RA * relLoc';
% (B) the no-transpose alternative:
RB = relOri;             tB = -RB * relLoc';
% (C) MATLAB's own pose->extrinsics conversion, mapped to premultiply:
[Rx_, tx_] = cameraPoseToExtrinsics(relOri, relLoc);
RC = Rx_';               tC = tx_(:);

% relativeCameraPose returns translation up to scale; impose the true baseline.
tA = unitScale(tA, b);  tB = unitScale(tB, b);  tC = unitScale(tC, b);

% Angles to ground truth.
aA = geoAngle(RA, R2gt);
aB = geoAngle(RB, R2gt);
aC = geoAngle(RC, R2gt);

% 3D recovery error driving each candidate through the real triangulator.
eA = recoverErr(K, intr1, intr2, RA, tA, R2gt, t2gt);
eB = recoverErr(K, intr1, intr2, RB, tB, R2gt, t2gt);

fprintf('    relativeCameraPose returned; comparing conventions to ground truth:\n');
fprintf('      (A) code  R_rel=relOri''      : R-angle %7.3f deg | 3D recover RMS %8.3f m\n', aA, eA);
fprintf('      (B) alt   R_rel=relOri        : R-angle %7.3f deg | 3D recover RMS %8.3f m\n', aB, eB);
fprintf('      (C) MATLAB cameraPoseToExtr.  : R-angle %7.3f deg  (blessed reference)\n', aC);

% The script (A) is correct iff it matches both ground truth and MATLAB's own (C).
codeMatchesTruth   = aA < 1e-2 && eA < 1e-3;
codeMatchesBlessed = geoAngle(RA, RC) < 1e-6;
fail = ~(codeMatchesTruth && codeMatchesBlessed);

if fail
    altIsCorrect = aB < 1e-2 && eB < 1e-3 && geoAngle(RB, RC) < 1e-6;
    fprintf('    VERDICT: script convention (A) is WRONG.\n');
    if altIsCorrect
        fprintf('             MATLAB''s own conversion agrees with (B) R_rel = relOri\n');
        fprintf('             (i.e. the transpose in the script is spurious).\n');
    end
else
    fprintf('    VERDICT: script convention (A) is CORRECT.\n');
end
report(~fail, sprintf('code-vs-truth %.3f deg / %.3e m; code-vs-MATLAB %.2e deg', ...
        aA, eA, geoAngle(RA, RC)));
end


% =========================================================================
% SUB-TEST 4 — fundamental-matrix / epipolar consistency
% =========================================================================
function fail = subFundamental(K, intr1, intr2, R2gt, t2gt)
fprintf('\n[4] FUNDAMENTAL  buildFundamentalMatrices epipolar residual (same extrinsics)\n');
cal.intrinsics = {intr1, intr2}; cal.R = {eye(3), R2gt}; cal.t = {[0;0;0], t2gt};
cal = buildFundamentalMatrices(cal, 2);
Xw = [ (rand(1,80)-0.5)*8 ; (rand(1,80)-0.5)*5 ; 6 + rand(1,80)*22 ];
x1 = projAll(K, eye(3), [0;0;0], Xw.');
x2 = projAll(K, R2gt,   t2gt,    Xw.');
res = zeros(1,80);
for k = 1:80
    p1  = [x1(k,:) 1].';  p2 = [x2(k,:) 1].';
    Fp1 = cal.F{1,2} * p1;                       % epipolar line in cam2
    res(k) = abs(p2.' * Fp1) / hypot(Fp1(1), Fp1(2));
end
m = max(res);
fail = m > 1e-6;
report(~fail, sprintf('max point-to-epipolar-line distance %.2e px', m));
end


% =========================================================================
% HELPERS
% =========================================================================
function report(pass, msg)
if pass, tag = 'PASS'; else, tag = 'FAIL'; end
fprintf('    [%s] %s\n', tag, msg);
end

function x = project(K, R, t, X)        % single point -> 1x2 pixel
p = K * (R * X + t);  x = (p(1:2)/p(3)).';
end

function px = projAll(K, R, t, Xw)      % Mx3 world -> Mx2 pixels
Xc = (R * Xw.' + t);
px = (K * Xc); px = (px(1:2,:) ./ px(3,:)).';
end

function d = depth(R, t, Xw)            % camera-frame Z for each Mx3 world point
Xc = R * Xw.' + t;  d = Xc(3,:).';
end

function v = unitScale(v, s)
v = v(:); n = norm(v); if n > 0, v = v / n * s; end
end

function a = geoAngle(Ra, Rb)
a = acosd(max(-1, min(1, (trace(Ra*Rb') - 1)/2)));
end

function e = recoverErr(K, intr1, intr2, R2, t2, R2gt, t2gt)
% Triangulate fresh points with candidate (R2,t2) through the real pipeline.
cal.intrinsics = {intr1, intr2}; cal.R = {eye(3), R2}; cal.t = {[0;0;0], t2};
cfg.N = 2; cfg.reprThreshold = 1e9;
Xt = [ -2 1 10 ; 0.5 -0.5 16 ; 2 1.2 13 ; -1 -1.5 22 ; 3 -2 9 ].';
nP = size(Xt,2);
g0 = struct('points', nan(2,2), 'camIds', [1 2], 'nViews', 2);
gr = repmat(g0, 1, nP);
for p = 1:nP
    gr(p).points(1,:) = project(K, eye(3), [0;0;0], Xt(:,p));
    gr(p).points(2,:) = project(K, R2gt,   t2gt,    Xt(:,p));   % TRUE projections
end
pts = triangulateGroups(gr, cal, cfg);                          % recovered with CANDIDATE
e = sqrt(mean(arrayfun(@(k) sum((pts(k).position - Xt(:,k).').^2), 1:nP)));
end

function R = quatAverage(R_all)
n = size(R_all,3); Q = zeros(4,n);
for k = 1:n, q = rotm2quat(R_all(:,:,k)); Q(:,k) = q(:); end
[V,~] = eig(Q*Q'); q = V(:,end); R = quat2rotm((q/norm(q)).');
end

function R = Rx(d), c=cosd(d); s=sind(d); R=[1 0 0;0 c -s;0 s c]; end
function R = Ry(d), c=cosd(d); s=sind(d); R=[c 0 s;0 1 0;-s 0 c]; end
function R = Rz(d), c=cosd(d); s=sind(d); R=[c -s 0;s c 0;0 0 1]; end
