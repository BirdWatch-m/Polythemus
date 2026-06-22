function epipolarOnRecording(recDir, nSample)
% EPIPOLARONRECORDING  Test a calibration's epipolar geometry on real scene data.
%
%   epipolarOnRecording(recDir, nSample)
%
%   Matches SURF features between the synchronized cam1/cam2 frames of a
%   recording (a static, textured scene) and measures how well those real
%   correspondences satisfy the calibration's epipolar geometry. No clicking,
%   no triangulation, no metric scale involved — this isolates the EXTRINSIC
%   rotation + translation DIRECTION (via the fundamental matrix) from
%   everything else.
%
%   Three numbers decide the cause of a large reprojection error:
%     (1) calibration-F error on true correspondences — the calibration itself.
%     (2) scene-F error on the same points — the BEST a single rigid geometry
%         can do for these two cameras/scene (independent of the calibration).
%     (3) rotation / translation-direction disagreement between the two.
%
%   Interpretation:
%     (1) small  (< ~2px)            -> extrinsic geometry is GOOD; a distance
%                                       error is scale or clicking, not epipolar.
%     (1) large, (2) small           -> the cameras CAN be related by a good
%                                       geometry but the CALIBRATION is wrong
%                                       (procedure/data) — see (3) for how wrong.
%     (1) large AND (2) large (>5px) -> NO single rigid geometry fits the pair:
%                                       physical cause (non-rigid mount between
%                                       frames, rolling shutter, parallax through
%                                       glass) — i.e. hardware, not calibration.
%
%   Uses the recording's bundled multiCamParams.mat if present, else the active
%   cfg.calFile. Read-only.
%
%   See also: buildFundamentalMatrices, validateCalibration, associateViews

if nargin < 1 || isempty(recDir)
    recDir = 'output/recordings/20260621_134509';
end
if nargin < 2 || isempty(nSample)
    nSample = 15;
end

calFile = fullfile(recDir, 'multiCamParams.mat');
if ~isfile(calFile), calFile = 'calibration/multiCamParams.mat'; end
S   = load(calFile);
cal = buildFundamentalMatrices(S.multiCamParams, 2);
F   = cal.F{1,2};
fprintf('Calibration: %s\n', calFile);
fprintf('  baseline |t{2}| = %.3f m,  cam2 centre [%.2f %.2f %.2f]\n', ...
        norm(cal.t{2}), -cal.R{2}.'*cal.t{2});

d1  = dir(fullfile(recDir, 'cam1', 'frame_*.tif'));
nF  = numel(d1);
idx = unique(round(linspace(1, nF, nSample)));
fprintf('Recording: %s  (%d frame pairs; sampling %d)\n', recDir, nF, numel(idx));

allP1 = []; allP2 = [];
for k = idx
    f1 = fullfile(recDir, 'cam1', sprintf('frame_%06d.tif', k));
    f2 = fullfile(recDir, 'cam2', sprintf('frame_%06d.tif', k));
    if ~isfile(f1) || ~isfile(f2), continue; end
    g1 = gray(imread(f1)); g2 = gray(imread(f2));
    p1 = detectSURFFeatures(g1, 'MetricThreshold', 300);
    p2 = detectSURFFeatures(g2, 'MetricThreshold', 300);
    if p1.Count < 20 || p2.Count < 20, continue; end
    [da, va] = extractFeatures(g1, p1);
    [db, vb] = extractFeatures(g2, p2);
    pr = matchFeatures(da, db, 'MatchThreshold', 50, 'MaxRatio', 0.7);
    if size(pr,1) < 8, continue; end
    allP1 = [allP1; va(pr(:,1)).Location]; %#ok<AGROW>
    allP2 = [allP2; vb(pr(:,2)).Location]; %#ok<AGROW>
end

nM = size(allP1,1);
fprintf('\nRaw SURF matches across sampled frames: %d\n', nM);
if nM < 30
    fprintf('Too few matches (scene may be mostly sky). Try another recording.\n');
    return;
end

% Robustly identify TRUE correspondences with a fresh scene fundamental matrix.
[Fscene, inl] = estimateFundamentalMatrix(allP1, allP2, ...
    'Method','RANSAC', 'NumTrials', 8000, 'DistanceThreshold', 1.0, 'Confidence', 99.99);
in1 = allP1(inl,:); in2 = allP2(inl,:);
fprintf('RANSAC inliers (consistent correspondences): %d / %d\n', sum(inl), nM);

eCal   = symEpi(in1, in2, F);        % (1) calibration geometry on true matches
eScene = symEpi(in1, in2, Fscene);   % (2) best achievable rigid geometry

fprintf('\n--- Epipolar error on %d true correspondences (pixels) ---\n', numel(eCal));
fprintf('  (1) CALIBRATION F : median %6.2f | 90th %6.2f | max %6.2f\n', ...
        median(eCal), pct(eCal,90), max(eCal));
fprintf('  (2) scene-fit  F  : median %6.2f | 90th %6.2f | max %6.2f\n', ...
        median(eScene), pct(eScene,90), max(eScene));

% (3) How far is the calibration's geometry from the scene's own geometry?
[relOri, relLoc] = relativeCameraPose(Fscene, cal.intrinsics{1}, cal.intrinsics{2}, in1, in2);
R_scene = relOri;                       % premultiply convention (verified)
t_scene = -R_scene * relLoc'; t_scene = t_scene / norm(t_scene);
t_cal   = cal.t{2} / norm(cal.t{2});
aR = geoAngle(R_scene, cal.R{2});
aT = acosd(min(1, abs(dot(t_scene, t_cal))));   % direction only (sign/scale free)
fprintf('\n--- Calibration vs scene-recovered geometry ---\n');
fprintf('  rotation disagreement        : %.2f deg\n', aR);
fprintf('  translation-direction disagree: %.2f deg\n', aT);

fprintf('\n--- Verdict ---\n');
mCal = median(eCal); mScene = median(eScene);
if mCal < 2
    fprintf('  Extrinsic epipolar geometry is GOOD on the real scene (median %.2fpx).\n', mCal);
    fprintf('  A distance error is then SCALE or click correspondence, not the pose.\n');
elseif mScene > 5
    fprintf('  No single rigid geometry fits this camera pair (scene-fit still %.2fpx).\n', mScene);
    fprintf('  Cause is PHYSICAL: non-rigid mount between captures, rolling shutter on\n');
    fprintf('  moving content, or refraction/parallax through the glass — not the math.\n');
else
    fprintf('  Cameras admit a good geometry (scene-fit %.2fpx) but the CALIBRATION\n', mScene);
    fprintf('  is off: %.2fpx median epipolar error, %.1f deg rotation / %.1f deg t-dir\n', mCal, aR, aT);
    fprintf('  from the scene''s own geometry. The extrinsic CAPTURE is the problem\n');
    fprintf('  (board coverage / pose diversity / origin ambiguity), not intrinsics.\n');
end
end


% ===================== helpers =====================
function g = gray(im)
if size(im,3) == 3, g = rgb2gray(im); else, g = im; end
end

function d = symEpi(P1, P2, F)
n = size(P1,1);
p1 = [P1 ones(n,1)].';  p2 = [P2 ones(n,1)].';
l2 = F * p1;  l1 = F.' * p2;
num = abs(sum(p2 .* l2, 1));
d   = 0.5 * (num ./ hypot(l1(1,:), l1(2,:)) + num ./ hypot(l2(1,:), l2(2,:)));
d   = d(:);
end

function a = geoAngle(Ra, Rb)
a = acosd(max(-1, min(1, (trace(Ra*Rb') - 1)/2)));
end

function v = pct(x, p)
xs = sort(x); v = xs(max(1, ceil(p/100*numel(xs))));
end
