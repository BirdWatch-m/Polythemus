% SALVAGECALIBRATION  Fix the extrinsics convention bug and recover metric t.
%
%   Run from the project root (F5 or paste into Command Window).
%   Edit the USER INPUTS block, then run.
%
%   What this does:
%     1. Fixes R{2} by transposing  (corrects postmultiply -> premultiply bug)
%     2. Recovers t direction via SURF feature matching on recording frames
%     3. Scales t to the midpoint of your baseline range
%     4. Spot-validates by letting you click a point in both cameras and
%        triangulating it — check that the reported distance looks plausible
%     5. Saves to calibration/multiCamParams_salvaged.mat
%
%   After running, if the validation looks reasonable:
%     copyfile('calibration/multiCamParams_salvaged.mat', 'calibration/multiCamParams.mat')
%   Then run processRecording with runMode = 'full'.

clc;
addpath(genpath('.'));

% =========================================================================
% USER INPUTS
% =========================================================================
recordingDir = 'output/recordings/20260617_174115';
baselineMin  = 2.0;   % lower bound on camera separation (m)
baselineMax  = 3.0;   % upper bound on camera separation (m)
calFile      = 'calibration/multiCamParams.mat';
outFile      = 'calibration/multiCamParams_salvaged.mat';
% =========================================================================

baseline = (baselineMin + baselineMax) / 2;

% ---- Load ----------------------------------------------------------------
fprintf('Loading session and calibration...\n');
sessionData = load(fullfile(recordingDir, 'session.mat'));
nFrames     = sessionData.session.nFrames;
cfg         = sessionData.session.cfg;
N           = cfg.N;

calData = load(calFile);
old     = calData.multiCamParams;

fprintf('  Recording: %d frames\n', nFrames);
fprintf('  Old |t{2}| = %.4f m   (for reference only — expected to be wrong)\n\n', ...
        norm(old.t{2}));

% =========================================================================
% STEP 1 — Fix R{2} by transposing
% =========================================================================
fprintf('=== Step 1: Fix R{2} ===\n');
% The calibration script treated extrinsics() as premultiply (X_cam = R*X_world + t)
% but it is postmultiply (X_cam = R'*X_world + t').  R stored = R_correct'.
R_fixed = old.R{2}';
fprintf('R{2} transposed.\n\n');

% =========================================================================
% STEP 2 — Recover t direction from recording frames via SURF matching
% =========================================================================
fprintf('=== Step 2: Recover t direction from recording frames ===\n');
fprintf('Scanning up to frame %d in steps of 10...\n\n', min(300, nFrames));

MIN_INLIERS  = 15;
FRAME_STRIDE = 10;

R_candidates  = {};
t_candidates  = {};
inlier_counts = [];

for k = 1 : FRAME_STRIDE : min(300, nFrames)
    f1 = fullfile(recordingDir, 'cam1', sprintf('frame_%06d.tif', k));
    f2 = fullfile(recordingDir, 'cam2', sprintf('frame_%06d.tif', k));
    if ~isfile(f1) || ~isfile(f2), continue; end

    img1 = imread(f1);
    img2 = imread(f2);
    if size(img1,3) == 3, img1 = rgb2gray(img1); end
    if size(img2,3) == 3, img2 = rgb2gray(img2); end

    pts1 = detectSURFFeatures(img1, 'MetricThreshold', 300);
    pts2 = detectSURFFeatures(img2, 'MetricThreshold', 300);
    if length(pts1) < 20 || length(pts2) < 20, continue; end

    [d1, pts1] = extractFeatures(img1, pts1);
    [d2, pts2] = extractFeatures(img2, pts2);
    pairs = matchFeatures(d1, d2, 'MatchThreshold', 50, 'MaxRatio', 0.7);
    if size(pairs,1) < 20, continue; end

    m1 = pts1(pairs(:,1));
    m2 = pts2(pairs(:,2));

    try
        [F, inliers] = estimateFundamentalMatrix(m1.Location, m2.Location, ...
            'Method', 'RANSAC', 'NumTrials', 2000, 'DistanceThreshold', 1.5);
    catch
        continue;
    end

    nIn = sum(inliers);
    fprintf('  Frame %4d: %3d matches -> %3d inliers\n', k, size(pairs,1), nIn);
    if nIn < MIN_INLIERS, continue; end

    try
        % relativeCameraPose is also postmultiply — apply the same fix:
        %   R12_correct  = relOri'
        %   cam2 centre in cam1 frame = relLoc' (unit, from F up-to-scale)
        %   t12_correct  = -R12_correct * relLoc'
        [relOri, relLoc] = relativeCameraPose(F, ...
            old.intrinsics{1}, old.intrinsics{2}, m1(inliers), m2(inliers));
    catch
        continue;
    end

    R_k = relOri';
    t_k = -R_k * relLoc';   % unit magnitude

    R_candidates{end+1}  = R_k;   %#ok<AGROW>
    t_candidates{end+1}  = t_k;   %#ok<AGROW>
    inlier_counts(end+1) = nIn;   %#ok<AGROW>
end

if isempty(R_candidates)
    fprintf('\nNo usable frames found (pure sky throughout?).\n');
    fprintf('3D not recoverable from this recording.\n');
    fprintf('Use processRecording with runMode = ''detect'' only.\n');
    return;
end

[~, best]     = max(inlier_counts);
R_from_frames = R_candidates{best};
t_unit        = t_candidates{best};
fprintf('\nSelected frame with %d inliers for t direction.\n', inlier_counts(best));

% =========================================================================
% STEP 3 — Sanity checks
% =========================================================================
fprintf('\n=== Step 3: Sanity checks ===\n');

% 3a: R_fixed (from file) vs R_from_frames (from feature matching) — should agree.
R_diff    = R_fixed * R_from_frames';
angle_deg = acosd(max(-1, min(1, (trace(R_diff) - 1) / 2)));
fprintf('R agreement (file-fix vs frames): %.2f deg\n', angle_deg);
if    angle_deg < 10, fprintf('  -> Good\n');
elseif angle_deg < 25, fprintf('  -> Moderate — feature noise is expected\n');
else,                  fprintf('  -> WARNING: large disagreement. Frame features may be unreliable.\n');
end

% 3b: t direction consistency across all candidate frames.
if numel(t_candidates) > 1
    maxAngle = 0;
    for a = 1:numel(t_candidates)
        for b = a+1:numel(t_candidates)
            c_ab     = dot(t_candidates{a}, t_candidates{b});
            maxAngle = max(maxAngle, acosd(max(-1, min(1, c_ab))));
        end
    end
    fprintf('t direction spread across %d usable frames: %.2f deg\n', ...
            numel(t_candidates), maxAngle);
end

% 3c: Camera 2 position in camera 1 frame — print for eyeballing.
% C2 = -R_fixed' * t_unit  (unit vector pointing from cam1 origin to cam2).
C2_dir = -R_fixed' * t_unit;
fprintf('Camera 2 direction in cam1 frame (unit): [%+.3f  %+.3f  %+.3f]\n', ...
        C2_dir(1), C2_dir(2), C2_dir(3));
fprintf('  (cam1 axes: X=right  Y=down  Z=forward into scene)\n');
fprintf('  For a side-by-side rig the X component should dominate.\n');

% =========================================================================
% STEP 4 — Scale t
% =========================================================================
fprintf('\n=== Step 4: Scale t to estimated baseline ===\n');
t_scaled = t_unit * baseline;
fprintf('Baseline mid-point = %.2f m  (range %.1f – %.1f m)\n', ...
        baseline, baselineMin, baselineMax);
fprintf('t{2} = [%+.4f  %+.4f  %+.4f] m\n', t_scaled(1), t_scaled(2), t_scaled(3));

% =========================================================================
% STEP 5 — Click-triangulation spot check on a recording frame
% =========================================================================
fprintf('\n=== Step 5: Click validation ===\n');
fprintf('You will click the same physical point in both camera frames.\n');
fprintf('Use a sharp, static feature visible in both — building corner, rooftop edge, etc.\n');
fprintf('The printed distance should look plausible; reprojection errors <3px is good.\n\n');

% Assemble the salvaged calibration now so triangulateGroups can use it.
salvaged             = old;
salvaged.R{2}        = R_fixed;
salvaged.t{2}        = t_scaled;

% Pick the frame with the most inliers for display — most likely to have
% scene content.  Fall back to frame 1 if needed.
[~, bestFrameCandidate] = max(inlier_counts);
% We didn't store which frame index each candidate came from, so just use 1.
% User can edit this line if frame 1 is pure sky.
valFrameIdx = 1;

f1v = fullfile(recordingDir, 'cam1', sprintf('frame_%06d.tif', valFrameIdx));
f2v = fullfile(recordingDir, 'cam2', sprintf('frame_%06d.tif', valFrameIdx));

clickedPixels = zeros(N, 2);
imgs = {imread(f1v), imread(f2v)};
for i = 1:N
    fig = figure('Name', sprintf('Click validation point — cam %d  (Enter when done)', i));
    imshow(imgs{i});
    title(sprintf('Cam %d — click the same static feature as in the other camera', i));
    [x, y] = ginput(1);
    clickedPixels(i,:) = [x y];
    fprintf('  Camera %d: clicked (%.1f, %.1f)\n', i, x, y);
    close(fig);
end

valGroup.camIds = 1:N;
valGroup.points = clickedPixels;   % row i = pixel in camera i

pt = triangulateGroups(valGroup, salvaged, cfg);

fprintf('\nTriangulated position : [%+.3f  %+.3f  %+.3f] m\n', pt.position);
fprintf('Distance from cam1    : %.3f m\n', norm(pt.position));
fprintf('Reprojection errors   : ');
fprintf('%.2fpx  ', pt.reprojErr); fprintf('\n');
fprintf('\nDoes that distance match what you clicked in the scene?\n');
fprintf('If the errors are >5px or the distance is wildly wrong, check Step 3 warnings.\n');

% =========================================================================
% STEP 6 — Save
% =========================================================================
fprintf('\n=== Step 6: Save ===\n');

multiCamParams = salvaged;
multiCamParams.salvageNote = sprintf( ...
    'Salvaged %s — R transposed (convention fix); t from frame feature matching, scaled to %.2fm (est. %.1f-%.1fm)', ...
    datestr(now), baseline, baselineMin, baselineMax);

save(outFile, 'multiCamParams');
fprintf('Saved: %s\n\n', outFile);
fprintf('If the Step 5 validation looked good, activate it:\n');
fprintf('  copyfile(''%s'', ''%s'')\n', outFile, calFile);
fprintf('Then run processRecording with runMode = ''full''.\n');
