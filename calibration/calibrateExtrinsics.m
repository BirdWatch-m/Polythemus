% CALIBRATEEXTRINSICS  Compute inter-camera geometry from a shared scene.
%
%   Script. Edit USER INPUTS below, then run with F5.
%
%   With all cameras mounted in their final positions, captures one
%   simultaneous frame per camera, matches visual features between
%   overlapping camera pairs, and recovers the rotation and translation of
%   every camera relative to camera 1 (which defines the world origin).
%   Saves the result to cfg.calFile.
%
%   Run this every time cameras are physically moved or re-aimed.
%   Intrinsic calibration (calibrateIntrinsics) does NOT need to be repeated.
%
%   USER INPUTS
%     intrinsicFiles  {1xN} paths to .mat files from calibrateIntrinsics
%     knownBaseline   physical distance between adjacent camera optical
%                     centres in metres (tape measure). Sets metric scale.
%                     Set to [] to save unit-scale translation.
%
%   OUTPUT
%     cfg.calFile — multiCamParams struct (stays in workspace as multiCamParams):
%       .intrinsics{i}  cameraParameters object for camera i
%       .R{i}           3x3 rotation matrix — orientation of camera i
%       .t{i}           3x1 translation vector — position of camera i
%                       Camera 1: R = eye(3), t = [0;0;0].
%
%   REQUIREMENTS
%     Cameras must share a textured scene region (buildings, rooftops, tree
%     lines). Blank sky will not produce enough feature matches. Brighter
%     daylight improves matching significantly.
%
%   ALTERNATIVE
%     Use calibrateExtrinsicsCheckerboard when the scene lacks texture.
%     That script uses a held checkerboard and derives metric scale from
%     squareSizeM directly — no knownBaseline measurement required.
%
%   See also: calibrateIntrinsics, calibrateExtrinsicsCheckerboard,
%             validateCalibration, initSystem

% =========================================================================
% USER INPUTS — edit these before running
% =========================================================================

clc; close all; clear;

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..')));

cfg = buildConfig();

intrinsicFiles = {'calibration/intrinsics_MY1_720.mat', ...
                  'calibration/intrinsics_LG1_720.mat'};

knownBaseline = 3.5;   % physical distance between camera optical centres, metres
                        % set to [] to save unit-scale translation

% =========================================================================

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
    intrinsics{i} = loaded.(fn{1});
    fprintf('Loaded intrinsics for camera %d (%.3fpx focal length).\n', ...
            i, mean(intrinsics{i}.FocalLength));
end

% -------------------------------------------------------------------------
% 2. CAPTURE SIMULTANEOUS FRAMES
% -------------------------------------------------------------------------

fprintf('\nOpening cameras for simultaneous capture...\n');
cams = cell(1, N);
for i = 1:N
    cams{i} = webcam(cfg.camIndices(i));
    cams{i}.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));
    applyCameraSettings(cams{i}, cfg);
end

% Live preview: stream all cameras simultaneously. Close the window to capture.
fprintf('Live preview — close the window to capture calibration frames.\n');
fig  = figure('Name', 'Extrinsic calibration — close to capture', 'NumberTitle', 'off');
hImg = gobjects(1, N);
for i = 1:N
    ax      = subplot(1, N, i);
    hImg(i) = imshow(snapshot(cams{i}), 'Parent', ax);
    title(ax, sprintf('Camera %d', i));
end

while ishandle(fig)
    try
        for i = 1:N
            set(hImg(i), 'CData', snapshot(cams{i}));
        end
        drawnow limitrate;
    catch
        break
    end
end

% Capture one frame per camera immediately after the window closes.
frames = cell(1, N);
for i = 1:N
    frames{i} = snapshot(cams{i});
end
cams = {};

% -------------------------------------------------------------------------
% 3. EXTRACT AND MATCH FEATURES FOR EACH CAMERA PAIR
% -------------------------------------------------------------------------
% Camera 1 is the world reference: R{1} = I, t{1} = 0.
% For every other camera i, we find its pose relative to camera 1 by:
%   (a) matching SURF features between camera 1 and camera i,
%   (b) estimating the fundamental matrix from those matches,
%   (c) recovering R and t from the fundamental matrix + intrinsics.

R = cell(1, N);
t = cell(1, N);
R{1} = eye(3);
t{1} = zeros(3, 1);

for i = 2:N
    ref = i - 1;
    fprintf('\nLocalising camera %d relative to camera %d...\n', i, ref);

    [R_rel, t_rel] = estimateRelativePose( ...
        frames{ref}, frames{i}, intrinsics{ref}, intrinsics{i});

    if ~isempty(knownBaseline)
        t_rel = t_rel * (knownBaseline / norm(t_rel));
    end

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
    C = -R{i}' * t{i};              % camera centre in world (cam1) frame
    [az, el] = rotationToAzEl(R{i});
    fprintf('Camera %d:  position [%.3f  %.3f  %.3f] m   azimuth %.1f deg   elevation %.1f deg\n', ...
            i, C(1), C(2), C(3), az, el);
end
fprintf('\nVerify baselines against your physical telemeter measurements.\n');

% -------------------------------------------------------------------------
% 5. ASSEMBLE AND SAVE
% -------------------------------------------------------------------------

multiCamParams.intrinsics   = intrinsics;
multiCamParams.R            = R;
multiCamParams.t            = t;
multiCamParams.capturedAt   = datestr(now);
multiCamParams.sourceFrames = frames;

if ~isfolder(fileparts(cfg.calFile))
    mkdir(fileparts(cfg.calFile));
end

save(cfg.calFile, 'multiCamParams');
fprintf('\nCalibration saved to: %s\n', cfg.calFile);
fprintf('Run validateCalibration to verify triangulation accuracy.\n');


% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function [R_rel, t_rel] = estimateRelativePose(frameRef, frameTarget, intrRef, intrTgt)
% Matches SURF features between two frames and recovers relative camera pose.

gryRef = rgb2gray(frameRef);
gryTgt = rgb2gray(frameTarget);

ptsRef = detectSURFFeatures(gryRef, 'MetricThreshold', 300);
ptsTgt = detectSURFFeatures(gryTgt, 'MetricThreshold', 300);

[descRef, ptsRef] = extractFeatures(gryRef, ptsRef);
[descTgt, ptsTgt] = extractFeatures(gryTgt, ptsTgt);

pairs = matchFeatures(descRef, descTgt, 'MatchThreshold', 50, 'MaxRatio', 0.7);

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

% relativeCameraPose's orientation already IS the world-to-camera premultiply
% R the pipeline uses — do NOT transpose it. Verified against MATLAB's own
% cameraPoseToExtrinsics and a synthetic round-trip (tests/testExtrinsicConventions.m).
% Contrast extrinsics(), whose R is the transpose and must be flipped (see
% calibrateExtrinsicsCheckerboard). relLoc is cam2's centre direction in cam1's frame.
[relOri, relLoc] = relativeCameraPose(F, intrRef, intrTgt, inlierRef, inlierTgt);
R_rel = relOri;
t_rel = -R_rel * relLoc';

end


function [az, el] = rotationToAzEl(R)
% Camera look direction in world frame = third row of R (world-to-camera premultiply).
az = atan2d(R(3,1), R(3,3));
el = -asind(R(3,2));
end
