function multiCamParams = calibrateExtrinsics(cfg, intrinsicFiles)
% CALIBRATEEXTRINSICS  Compute inter-camera geometry from a shared scene.
%
%   multiCamParams = calibrateExtrinsics(cfg, intrinsicFiles)
%
%   With all cameras mounted in their final positions, this function captures
%   one simultaneous frame per camera, matches visual features between
%   overlapping camera pairs, and recovers the rotation and translation of
%   every camera relative to camera 1 (which defines the world origin).
%
%   Run this every time cameras are physically moved or re-aimed.
%   Intrinsic calibration (calibrateIntrinsics) does NOT need to be repeated.
%
%   INPUTS
%     cfg            — struct from buildConfig()
%     intrinsicFiles — {1 x N} cell of strings, paths to the .mat files
%                      saved by calibrateIntrinsics for each camera.
%                      e.g. {'calibration/intrinsics_cam1.mat',
%                             'calibration/intrinsics_cam2.mat',
%                             'calibration/intrinsics_cam3.mat'}
%
%   OUTPUT
%     multiCamParams — struct saved to cfg.calFile, containing:
%       .intrinsics{i}  cameraIntrinsics object for camera i
%       .R{i}           3x3 rotation matrix — orientation of camera i
%       .t{i}           3x1 translation vector — position of camera i
%                       Camera 1 has R = eye(3), t = [0;0;0] by definition.
%       .F{i,j}         3x3 fundamental matrix for pair (i,j); added by
%                       initSystem on load, not stored here.
%
%   COORDINATE CONVENTION
%     World origin = optical centre of camera 1.
%     X axis = rightward in camera 1's view.
%     Y axis = downward in camera 1's view.
%     Z axis = into the scene (depth).
%
%   REQUIREMENTS
%     - Cameras must have overlapping fields of view with a distant scene
%       (buildings, rooftops, tree lines) containing rich texture.
%     - Each camera must share visible features with at least one already-
%       localised camera (camera 1 is always localised first).
%     - Brighter daylight conditions improve feature matching significantly.
%
%   VALIDATION
%     After saving, run validateCalibration.m to measure triangulation
%     accuracy against a known physical distance.
%
%   See also: calibrateIntrinsics, validateCalibration, initSystem

N = cfg.N;

if numel(intrinsicFiles) ~= N
    error('calibrateExtrinsics:fileMismatch', ...
          'intrinsicFiles has %d entries but cfg.N = %d.', ...
          numel(intrinsicFiles), N);
end

% -------------------------------------------------------------------------
% 1. LOAD INTRINSICS
% -------------------------------------------------------------------------

intrinsics = cell(1, N);
for i = 1:N
    if ~isfile(intrinsicFiles{i})
        error('calibrateExtrinsics:missingFile', ...
              'Intrinsics file not found: %s', intrinsicFiles{i});
    end
    loaded = load(intrinsicFiles{i});
    fn = fieldnames(loaded);
    intrinsics{i} = loaded.(fn{1});   % load whatever variable is in the file
    fprintf('Loaded intrinsics for camera %d (%.3fpx focal length).\n', ...
            i, mean(intrinsics{i}.FocalLength));
end

% -------------------------------------------------------------------------
% 2. CAPTURE SIMULTANEOUS FRAMES
% -------------------------------------------------------------------------

fprintf('\nOpening cameras for simultaneous capture...\n');
cams = cell(1, N);
for i = 1:N
    cams{i} = webcam(i);
    cams{i}.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));
end

fprintf('Press any key to capture from all cameras simultaneously.\n');
figure('Name','Extrinsic calibration — preview'); pause;

frames = cell(1, N);
for i = 1:N
    frames{i} = snapshot(cams{i});
end

% Close cameras immediately; we work from the captured frames.
for i = 1:N, clear cams{i}; end

% Show captured frames side by side for visual confirmation.
figure('Name', 'Captured frames — verify scene overlap');
for i = 1:N
    subplot(1, N, i);
    imshow(frames{i});
    title(sprintf('Camera %d', i));
end
fprintf('Check frames for scene overlap. Close figure to continue.\n');
uiwait(gcf);

% -------------------------------------------------------------------------
% 3. EXTRACT AND MATCH FEATURES FOR EACH CAMERA PAIR
% -------------------------------------------------------------------------
% Camera 1 is the world reference: R{1} = I, t{1} = 0.
% For every other camera i, we find its pose relative to camera 1 by:
%   (a) matching features between camera 1 and camera i,
%   (b) estimating the fundamental matrix from those matches,
%   (c) recovering R and t from the fundamental matrix + intrinsics.
%
% If cameras i and 1 share insufficient overlap (e.g. camera 3 vs camera 1
% across a large baseline), we chain: localise camera 2 from camera 1,
% then localise camera 3 from camera 2, then compose the transforms.

R = cell(1, N);
t = cell(1, N);

% Camera 1 is the world origin by definition.
R{1} = eye(3);
t{1} = zeros(3, 1);

% Determine localisation order: greedy — process cameras in index order.
% If a pair has insufficient matches, the function will error with advice.
for i = 2:N

    % Find the already-localised camera with best expected overlap with i.
    % Simple heuristic: use the nearest lower-index camera.
    ref = i - 1;

    fprintf('\nLocalising camera %d relative to camera %d...\n', i, ref);

    [R_rel, t_rel] = estimateRelativePose( ...
        frames{ref}, frames{i}, intrinsics{ref}, intrinsics{i});

    % Compose with reference camera's pose to get pose in world frame.
    % If ref = 1, R_rel and t_rel are already world-frame.
    % If ref > 1, we compose: world <- ref <- i.
    R{i} = R_rel * R{ref};
    t{i} = R_rel * t{ref} + t_rel;

    fprintf('Camera %d localised. Baseline from cam1: %.3fm\n', ...
            i, norm(t{i} - t{1}));
end

% -------------------------------------------------------------------------
% 4. PRINT GEOMETRY SUMMARY
% -------------------------------------------------------------------------

fprintf('\n--- Camera geometry summary (world origin = camera 1) ---\n');
for i = 1:N
    [az, el] = rotationToAzEl(R{i});
    fprintf('Camera %d:  position [%.3f  %.3f  %.3f] m   azimuth %.1f deg   elevation %.1f deg\n', ...
            i, t{i}(1), t{i}(2), t{i}(3), az, el);
end
fprintf('\nVerify baselines against your physical telemeter measurements.\n');

% -------------------------------------------------------------------------
% 5. ASSEMBLE AND SAVE
% -------------------------------------------------------------------------

multiCamParams.intrinsics = intrinsics;
multiCamParams.R          = R;
multiCamParams.t          = t;
multiCamParams.capturedAt = datestr(now);
multiCamParams.sourceFrames = frames;   % stored for later inspection

if ~isfolder(fileparts(cfg.calFile))
    mkdir(fileparts(cfg.calFile));
end

save(cfg.calFile, 'multiCamParams');
fprintf('\nCalibration saved to: %s\n', cfg.calFile);
fprintf('Run validateCalibration(cfg) to verify triangulation accuracy.\n');

end


% =========================================================================
% LOCAL HELPERS
% =========================================================================

function [R_rel, t_rel] = estimateRelativePose(frameRef, frameTarget, intrRef, intrTgt)
% Matches SURF features between two frames and recovers relative camera pose.

gryRef = rgb2gray(frameRef);
gryTgt = rgb2gray(frameTarget);

% Detect SURF feature points in both frames.
ptsRef = detectSURFFeatures(gryRef, 'MetricThreshold', 600);
ptsTgt = detectSURFFeatures(gryTgt, 'MetricThreshold', 600);

% Extract feature descriptors at each detected point.
[descRef, ptsRef] = extractFeatures(gryRef, ptsRef);
[descTgt, ptsTgt] = extractFeatures(gryTgt, ptsTgt);

% Match descriptors between the two frames; keep only strong matches.
pairs = matchFeatures(descRef, descTgt, 'MatchThreshold', 50, 'MaxRatio', 0.6);

matchedRef = ptsRef(pairs(:,1));
matchedTgt = ptsTgt(pairs(:,2));

if size(pairs,1) < 20
    error('estimateRelativePose:tooFewMatches', ...
          ['Only %d feature matches found between this camera pair.\n' ...
           'Ensure cameras share a textured scene region (not blank sky).\n' ...
           'Try shooting on a day with better light, or adjust camera angles.'], ...
          size(pairs,1));
end

fprintf('  %d feature matches found.\n', size(pairs,1));

% Estimate fundamental matrix with RANSAC to reject false matches.
[F, inliers] = estimateFundamentalMatrix( ...
    matchedRef.Location, matchedTgt.Location, ...
    'Method',    'RANSAC', ...
    'NumTrials', 2000, ...
    'DistanceThreshold', 1.5);

inlierRef = matchedRef(inliers);
inlierTgt = matchedTgt(inliers);

fprintf('  %d inliers after RANSAC (%.0f%%).\n', ...
        sum(inliers), 100*sum(inliers)/numel(inliers));

if sum(inliers) < 12
    error('estimateRelativePose:tooFewInliers', ...
          'Only %d inliers remain. Scene may lack enough parallax or texture.', ...
          sum(inliers));
end

% Recover rotation and translation from fundamental matrix + intrinsics.
[R_rel, t_rel] = relativeCameraPose(F, intrRef, intrTgt, ...
                                     inlierRef, inlierTgt);

% relativeCameraPose returns t up to an unknown scale — it can only
% determine direction, not magnitude, from images alone.
% Absolute scale is fixed when we know the physical baseline.
% We leave t_rel as unit-scale here; validateCalibration handles scaling.
t_rel = t_rel';   % ensure column vector

end


function [az, el] = rotationToAzEl(R)
% Extracts approximate azimuth and elevation angles from a rotation matrix.
% Useful only for sanity-checking output; not used in computation.
az = atan2d(R(1,3), R(3,3));
el = -asind(R(2,3));
end
