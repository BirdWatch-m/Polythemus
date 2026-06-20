function montecarloExtrinsicsPooled(recDir, Ks, nRuns)
% MONTECARLOEXTRINSICSPOOLED  Show the pooled SURF estimator collapses to one mode.
%
%   montecarloExtrinsicsPooled(recDir, Ks, nRuns)
%
%   Sweeps the number of pooled frames K. For each K it pools SURF matches over
%   K frames sampled across recDir (once, deterministic), then runs the PRODUCTION
%   estimator relativePoseFromMatches nRuns times — the only varying ingredient is
%   the RANSAC seed, exactly as in repeated calibrateExtrinsics runs. It reports
%   how many distinct rotation modes appear, the dominant mode's share, the
%   rotation spread, and the median epipolar error of each run's geometry against
%   a robust multi-frame reference. As K grows the modes collapse to one and the
%   error drops to the ~0.5px floor.
%
%   Compare K = 1 here (the old single-snapshot behaviour: several modes) with
%   K = 20-40 (one stable mode).
%
%   Read-only. Uses the 720p intrinsics and cfg.calExtrinsics settings.
%
%   See also: calibrateExtrinsics, poolPairMatches, relativePoseFromMatches,
%             montecarloExtrinsics, epipolarOnRecording

if nargin < 1 || isempty(recDir), recDir = 'output/recordings/20260619_201550'; end
if nargin < 2 || isempty(Ks),     Ks     = [1 2 5 10 20 40]; end
if nargin < 3 || isempty(nRuns),  nRuns  = 50; end

cfg = buildConfig();
p   = cfg.calExtrinsics;
intr1 = loadIntr('calibration/intrinsics_MY1_720.mat');
intr2 = loadIntr('calibration/intrinsics_LG1_720.mat');
K1 = intr1.IntrinsicMatrix.'; K2 = intr2.IntrinsicMatrix.';

nF = numel(dir(fullfile(recDir, 'cam1', 'frame_*.tif')));
fprintf('========================================================\n');
fprintf(' POOLED-ESTIMATOR STABILITY SWEEP  (%d runs per K)\n', nRuns);
fprintf('========================================================\n');
fprintf('Recording: %s  (%d frame pairs)\n', recDir, nF);

% ---- Robust reference (truth proxy) from many frames ----
refIdx = unique(round(linspace(1, nF, 25)));
[P1, P2] = poolFromIdx(recDir, refIdx, p);
[~, inl] = estimateFundamentalMatrix(P1, P2, 'Method','RANSAC', ...
    'NumTrials', 12000, 'DistanceThreshold', 1.0, 'Confidence', 99.99);
refIn1 = P1(inl,:); refIn2 = P2(inl,:);
fprintf('Reference: %d pooled / %d inliers over %d frames.\n\n', ...
        size(P1,1), sum(inl), numel(refIdx));

fprintf('%3s %8s %9s %7s %9s %12s %11s\n', ...
        'K','pooled','inlier%','#modes','domShare','rotSpread°','poolEpi px');

for K = Ks
    idx = unique(round(linspace(1, nF, K)));
    [m1, m2] = poolFromIdx(recDir, idx, p);

    Rs = zeros(3,3,nRuns); poolE = zeros(1,nRuns); ifr = zeros(1,nRuns);
    for r = 1:nRuns
        [R_rel, t_rel, info] = relativePoseFromMatches(m1, m2, intr1, intr2, p);
        Rs(:,:,r) = R_rel;
        ifr(r)    = info.inlierFrac;
        Fk        = Frel(R_rel, t_rel, K1, K2);
        poolE(r)  = median(symEpi(refIn1, refIn2, Fk));
    end

    [clid, cents] = clusterRot(Rs, 3.0);
    sizes   = accumarray(clid(:), 1);
    [domN, dom] = max(sizes);
    spread  = max(arrayfun(@(r) geoAngle(Rs(:,:,r), cents{dom}), 1:nRuns));

    fprintf('%3d %8d %8.0f%% %7d %8.0f%% %11.2f %11.2f\n', ...
            K, size(m1,1), 100*mean(ifr), numel(cents), ...
            100*domN/nRuns, spread, median(poolE));
end

fprintf('\nRead: #modes -> 1 and poolEpi -> ~0.5px as K grows means the pooled\n');
fprintf('estimator is stable and correct; K=1 reproduces the single-snapshot lottery.\n');
end


% ============================ helpers ============================
function [m1, m2] = poolFromIdx(recDir, idx, p)
fr1 = {}; fr2 = {};
for j = 1:numel(idx)
    k  = idx(j);
    f1 = fullfile(recDir, 'cam1', sprintf('frame_%06d.tif', k));
    f2 = fullfile(recDir, 'cam2', sprintf('frame_%06d.tif', k));
    if ~isfile(f1) || ~isfile(f2), continue; end
    fr1{end+1} = imread(f1); %#ok<AGROW>
    fr2{end+1} = imread(f2); %#ok<AGROW>
end
[m1, m2] = poolPairMatches(fr1, fr2, p);
end

function obj = loadIntr(f)
S = load(f); fn = fieldnames(S); obj = S.(fn{1});
end

function F = Frel(R, t, K1, K2)
tx = [0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0];
F  = (K2 \ eye(3)).' * (tx * R) / K1;
end

function d = symEpi(P1, P2, F)
n = size(P1,1);
p1 = [P1 ones(n,1)].'; p2 = [P2 ones(n,1)].';
l2 = F*p1; l1 = F.'*p2;
num = abs(sum(p2.*l2, 1));
d = 0.5*(num./hypot(l1(1,:),l1(2,:)) + num./hypot(l2(1,:),l2(2,:)));
d = d(:);
end

function [clid, cents] = clusterRot(Rs, thrDeg)
n = size(Rs,3); clid = zeros(1,n); cents = {};
for k = 1:n
    placed = false;
    for c = 1:numel(cents)
        if geoAngle(Rs(:,:,k), cents{c}) < thrDeg, clid(k) = c; placed = true; break; end
    end
    if ~placed, cents{end+1} = Rs(:,:,k); clid(k) = numel(cents); end %#ok<AGROW>
end
end

function a = geoAngle(Ra, Rb)
a = acosd(max(-1, min(1, (trace(Ra*Rb') - 1)/2)));
end
