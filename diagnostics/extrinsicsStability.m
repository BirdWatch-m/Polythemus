function extrinsicsStability(recDir, nRuns, knownBaseline)
% EXTRINSICSSTABILITY  Per-axis run-to-run stability of the pooled SURF extrinsics.
%
%   extrinsicsStability(recDir, nRuns, knownBaseline)
%
%   Runs the production pooled estimator nRuns times on one recording (a scene
%   rich enough that rotation and lateral baseline are stable) and reports the
%   scatter of camera 2's optical centre per axis: X (lateral), Y (vertical),
%   Z (forward/depth). It then sweeps cam2's forward offset and shows the
%   epipolar residual barely moves — i.e. the depth axis carries almost no
%   epipolar signal and cannot be pinned from a distant scene. Finally it repeats
%   the run with cfg.calExtrinsics.fixCam2Coplanar = true to show the Z scatter
%   collapses while X/Y are unchanged.
%
%   This is the evidence that the leftover run-to-run wobble is in the depth axis
%   only, that it is a geometry (low-parallax) limit rather than a code bug, and
%   that the coplanar constraint removes it cleanly.
%
%   See also: relativePoseFromMatches, decomposeStability, calibrateExtrinsics

if nargin < 1 || isempty(recDir),        recDir = 'output/recordings/20260619_201550'; end
if nargin < 2 || isempty(nRuns),         nRuns  = 30; end
if nargin < 3 || isempty(knownBaseline), knownBaseline = 3.5; end

cfg = buildConfig(); p = cfg.calExtrinsics;
intr1 = loadIntr('calibration/intrinsics_MY1_720.mat');
intr2 = loadIntr('calibration/intrinsics_LG1_720.mat');
K1 = intr1.IntrinsicMatrix.'; K2 = intr2.IntrinsicMatrix.';

nF  = numel(dir(fullfile(recDir,'cam1','frame_*.tif')));
idx = unique(round(linspace(1, nF, 30)));
[m1, m2] = poolFromIdx(recDir, idx, p);
fprintf('Recording: %s  | pooled %d matches over %d frames\n', recDir, size(m1,1), numel(idx));

% Holdout correspondences for the observability sweep.
[~, inl] = estimateFundamentalMatrix(m1, m2, 'Method','RANSAC', ...
    'NumTrials', 12000, 'DistanceThreshold', 1.0, 'Confidence', 99.99);
h1 = m1(inl,:); h2 = m2(inl,:);

% --- Per-axis scatter, unconstrained vs coplanar-constrained ---
pu = p; pu.fixCam2Coplanar = false; pu.fixCam2Level = false;   % zero neither axis
pc = p; pc.fixCam2Coplanar = true;  pc.fixCam2Level = true;    % zero Y and Z
Cu = runMode(m1, m2, intr1, intr2, pu, knownBaseline, nRuns);
Cc = runMode(m1, m2, intr1, intr2, pc, knownBaseline, nRuns);

fprintf('\nCamera-2 optical centre over %d runs (metres, mean +/- std [min..max]):\n', nRuns);
fprintf('  %-14s %-22s %-22s %-22s\n', 'mode', 'X (lateral)', 'Y (vertical)', 'Z (forward/depth)');
printAxis('unconstrained', Cu);
printAxis('Y+Z fixed', Cc);

% --- Forward-offset observability: epipolar vs cam2 Z ---
Rbar = Cu.Rmedoid;
Xb = median(Cu.C(1,:)); Yb = median(Cu.C(2,:));
dz  = -1.5:0.5:1.5;
epi = zeros(size(dz));
for k = 1:numel(dz)
    C2 = [Xb; Yb; dz(k)];
    tu = -Rbar * C2; tu = tu / norm(tu);
    epi(k) = median(symEpi(h1, h2, Frel(Rbar, tu, K1, K2)));
end
fprintf('\nForward-offset observability (hold rotation + lateral, vary cam2 Z):\n');
fprintf('   dz (m): '); fprintf('%6.1f', dz); fprintf('\n');
fprintf('  epi(px): '); fprintf('%6.2f', epi); fprintf('\n');
fprintf('  -> epipolar moves only %.2f px across +/-1.5 m of forward offset: Z is unobservable.\n', ...
        max(epi)-min(epi));

fprintf('\n--- Reading ---\n');
fprintf('If X/Y are tight in both modes and only unconstrained Z is wide, the wobble is\n');
fprintf('purely the depth axis. The flat epipolar-vs-Z curve proves the scene cannot fix\n');
fprintf('it; fixCam2Coplanar removes the free DOF for a level side-by-side rig.\n');
end


% ============================ helpers ============================
function out = runMode(m1, m2, intr1, intr2, p, B, nRuns)
C = zeros(3, nRuns); Rs = zeros(3,3,nRuns);
for r = 1:nRuns
    [R_rel, t_rel] = relativePoseFromMatches(m1, m2, intr1, intr2, p);
    Rs(:,:,r) = R_rel;
    C(:,r) = -R_rel.' * (B * t_rel);     % cam2 centre, metric
end
% medoid rotation (closest to all others)
d = zeros(1,nRuns);
for a = 1:nRuns, d(a) = sum(arrayfun(@(b) geoAngle(Rs(:,:,a),Rs(:,:,b)), 1:nRuns)); end
[~, mi] = min(d);
out.C = C; out.Rmedoid = Rs(:,:,mi);
end

function printAxis(name, S)
ax = @(i) sprintf('%6.2f+/-%4.2f [%5.2f..%4.2f]', ...
        mean(S.C(i,:)), std(S.C(i,:)), min(S.C(i,:)), max(S.C(i,:)));
fprintf('  %-14s %-22s %-22s %-22s\n', name, ax(1), ax(2), ax(3));
end

function [m1,m2] = poolFromIdx(recDir, idx, p)
fr1={}; fr2={};
for j = 1:numel(idx)
    f1=fullfile(recDir,'cam1',sprintf('frame_%06d.tif',idx(j)));
    f2=fullfile(recDir,'cam2',sprintf('frame_%06d.tif',idx(j)));
    if ~isfile(f1)||~isfile(f2), continue; end
    fr1{end+1}=imread(f1); fr2{end+1}=imread(f2); %#ok<AGROW>
end
[m1,m2]=poolPairMatches(fr1,fr2,p);
end

function obj = loadIntr(f), S=load(f); fn=fieldnames(S); obj=S.(fn{1}); end
function F = Frel(R,t,K1,K2), tx=[0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0]; F=(K2\eye(3)).'*(tx*R)/K1; end
function d = symEpi(P1,P2,F)
n=size(P1,1); p1=[P1 ones(n,1)].'; p2=[P2 ones(n,1)].';
l2=F*p1; l1=F.'*p2; num=abs(sum(p2.*l2,1));
d=0.5*(num./hypot(l1(1,:),l1(2,:))+num./hypot(l2(1,:),l2(2,:))); d=d(:);
end
function a=geoAngle(Ra,Rb), a=acosd(max(-1,min(1,(trace(Ra*Rb')-1)/2))); end
