function decomposeStability(recDir, nRuns)
% DECOMPOSESTABILITY  Localise the SURF-extrinsics instability: F vs pose decomposition.
%
%   Separates three possible causes of the run-to-run pose scatter:
%     (1) relativeCameraPose nondeterminism  — decompose ONE fixed F many times.
%     (2) fundamental-matrix instability      — raw RANSAC F on a disjoint holdout.
%     (3) decomposition ill-conditioning      — raw F good on holdout but the
%                                               recovered pose's F is not.
%   Plus the parallax (baseline/depth) that governs whether a metric pose is
%   recoverable at all from this geometry.
%
%   Estimation matches on frame set A; a robust reference (F + inliers) is built
%   on a DISJOINT frame set B, so all holdout numbers are out-of-sample.
%
%   See also: montecarloExtrinsicsPooled, relativePoseFromMatches, epipolarOnRecording

if nargin < 1 || isempty(recDir), recDir = 'output/recordings/20260619_201550'; end
if nargin < 2 || isempty(nRuns),  nRuns  = 50; end

cfg = buildConfig(); p = cfg.calExtrinsics;
intr1 = loadIntr('calibration/intrinsics_MY1_720.mat');
intr2 = loadIntr('calibration/intrinsics_LG1_720.mat');
K1 = intr1.IntrinsicMatrix.'; K2 = intr2.IntrinsicMatrix.';

nF = numel(dir(fullfile(recDir,'cam1','frame_*.tif')));
idxAll = round(linspace(1, nF, 40));
A = idxAll(1:2:end); B = idxAll(2:2:end);   % disjoint frame sets

[a1, a2] = poolFromIdx(recDir, A, p);        % estimation matches
[b1, b2] = poolFromIdx(recDir, B, p);        % holdout matches
[Fref, inl] = estimateFundamentalMatrix(b1, b2, 'Method','RANSAC', ...
    'NumTrials', 12000, 'DistanceThreshold', 1.0, 'Confidence', 99.99);
hin1 = b1(inl,:); hin2 = b2(inl,:);
fprintf('Estimation pool: %d matches | Holdout: %d matches, %d inliers\n\n', ...
        size(a1,1), size(b1,1), sum(inl));

% --- (1) Is relativeCameraPose deterministic on a FIXED F? ---
[Ffix, fi] = estimateFundamentalMatrix(a1, a2, 'Method','RANSAC', ...
    'NumTrials', p.ransacNumTrials, 'DistanceThreshold', p.ransacDistance, 'Confidence', p.ransacConfidence);
fi1 = a1(fi,:); fi2 = a2(fi,:);
Rdet = zeros(3,3,10);
for k = 1:10
    [ro, ~] = relativeCameraPose(Ffix, intr1, intr2, fi1, fi2);
    Rdet(:,:,k) = ro;
end
detSpread = max(arrayfun(@(k) geoAngle(Rdet(:,:,k), Rdet(:,:,1)), 1:10));
fprintf('(1) relativeCameraPose determinism on a FIXED F: rotation spread %.3f deg over 10 calls\n', detSpread);
if detSpread > 0.5
    fprintf('    -> relativeCameraPose is NONDETERMINISTic (internal randomness).\n');
else
    fprintf('    -> deterministic given F; scatter must come from F varying across RANSAC.\n');
end

% --- (2)+(3) Across RANSAC seeds: raw-F vs pose-F on the holdout ---
epiRawF  = zeros(1,nRuns);   % holdout epipolar under the raw RANSAC F  (F quality)
epiPoseF = zeros(1,nRuns);   % holdout epipolar under the pose's rebuilt F (pose quality)
Rs = zeros(3,3,nRuns);
for r = 1:nRuns
    [Fr, ir] = estimateFundamentalMatrix(a1, a2, 'Method','RANSAC', ...
        'NumTrials', p.ransacNumTrials, 'DistanceThreshold', p.ransacDistance, 'Confidence', p.ransacConfidence);
    epiRawF(r) = median(symEpi(hin1, hin2, Fr));
    [ro, rl] = relativeCameraPose(Fr, intr1, intr2, a1(ir,:), a2(ir,:));
    Rs(:,:,r) = ro; td = -ro*rl.'; td = td/norm(td);
    epiPoseF(r) = median(symEpi(hin1, hin2, Frel(ro, td, K1, K2)));
end
[~, cents] = clusterRot(Rs, 3.0);
fprintf('\n(2) Raw RANSAC F on holdout : median %.2f px | range %.2f..%.2f  (F quality across seeds)\n', ...
        median(epiRawF), min(epiRawF), max(epiRawF));
fprintf('(3) Pose-rebuilt F on holdout: median %.2f px | range %.2f..%.2f  (%d rotation modes)\n', ...
        median(epiPoseF), min(epiPoseF), max(epiPoseF), numel(cents));

% --- Parallax / conditioning (scale-free, in baseline units) ---
[ro, rl] = relativeCameraPose(Fref, intr1, intr2, hin1, hin2);
Rr = ro; tr = -Rr*rl.'; tr = tr/norm(tr);     % unit baseline
[depthBU, conv] = parallax(hin1, hin2, K1, K2, Rr, tr);
fprintf('\nParallax (geometry conditioning):\n');
fprintf('  median scene depth      : %.1f baseline-lengths (so ~%.0f m at 3 m baseline)\n', ...
        depthBU, 3*depthBU);
fprintf('  median convergence angle: %.2f deg between the two rays\n', conv);
if conv < 3
    fprintf('  -> LOW parallax: the F->pose decomposition and metric depth are ill-conditioned.\n');
end

fprintf('\n--- Reading ---\n');
fprintf('If (1) deterministic, (2) tight & low, but (3) high/multi-modal: the FUNDAMENTAL\n');
fprintf('MATRIX is fine for epipolar use, but decomposing it to a metric pose is unstable\n');
fprintf('because parallax is too low. That is geometry (baseline vs range), not a code bug.\n');
end


% ============================ helpers ============================
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

function [depthBU, convDeg] = parallax(P1, P2, K1, K2, R, t)
% Linear triangulation in baseline units; returns median depth and ray angle.
M1 = K1*[eye(3) zeros(3,1)];
M2 = K2*[R t];
n = size(P1,1); X = zeros(3,n);
for i = 1:n
    A = [P1(i,1)*M1(3,:)-M1(1,:); P1(i,2)*M1(3,:)-M1(2,:);
         P2(i,1)*M2(3,:)-M2(1,:); P2(i,2)*M2(3,:)-M2(2,:)];
    [~,~,V] = svd(A,0); Xh = V(:,end); X(:,i) = Xh(1:3)/Xh(4);
end
C2 = -R.'*t;
d1 = X ./ vecnorm(X);
d2 = (X - C2) ./ vecnorm(X - C2);
ang = acosd(max(-1,min(1,sum(d1.*d2,1))));
depthBU = median(X(3,:));
convDeg = median(ang);
end

function obj = loadIntr(f), S=load(f); fn=fieldnames(S); obj=S.(fn{1}); end
function F = Frel(R,t,K1,K2), tx=[0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0]; F=(K2\eye(3)).'*(tx*R)/K1; end
function d = symEpi(P1,P2,F)
n=size(P1,1); p1=[P1 ones(n,1)].'; p2=[P2 ones(n,1)].';
l2=F*p1; l1=F.'*p2; num=abs(sum(p2.*l2,1));
d=0.5*(num./hypot(l1(1,:),l1(2,:))+num./hypot(l2(1,:),l2(2,:))); d=d(:);
end
function [clid,cents]=clusterRot(Rs,thr)
n=size(Rs,3); clid=zeros(1,n); cents={};
for k=1:n, pl=false;
  for c=1:numel(cents), if geoAngle(Rs(:,:,k),cents{c})<thr, clid(k)=c; pl=true; break; end, end
  if ~pl, cents{end+1}=Rs(:,:,k); clid(k)=numel(cents); end %#ok<AGROW>
end
end
function a=geoAngle(Ra,Rb), a=acosd(max(-1,min(1,(trace(Ra*Rb')-1)/2))); end
