% CALIBRATEEXTRINSICS  Inter-camera geometry from a shared scene (multi-frame pooled).
%
%   Script. Edit USER INPUTS below, then run with F5.
%
%   With all cameras mounted in their final positions, captures a few seconds of
%   frames per camera (cfg.calExtrinsics.captureSeconds), POOLS SURF
%   correspondences across all of them, fits ONE robust fundamental matrix per
%   overlapping camera pair, and recovers the rotation and translation of every
%   camera relative to camera 1 (the world origin). Saves the result to
%   cfg.calFile.
%
%   WHY POOLED (not a single snapshot): on a repetitive scene (building façades)
%   a single frame pair has a high mismatch rate, and RANSAC locks onto one of
%   several competing consensus sets — so a single-pair estimate is multi-modal
%   and jumps between a few different poses run-to-run. Genuine correspondences
%   recur across frames while façade mismatches do not, so pooling makes the
%   true geometry the dominant consensus and the estimate stable. See
%   diagnostics/montecarloExtrinsicsPooled for the evidence.
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
%       .calibDiag      per-pair diagnostics (pooled/inlier counts, epipolar px)
%
%   REQUIREMENTS
%     Cameras must share a textured scene region (buildings, rooftops, tree
%     lines). Blank sky will not produce enough feature matches. Brighter
%     daylight improves matching significantly. Validate with epipolarOnRecording
%     or validateCalibration after running.
%
%   ALTERNATIVE
%     Use calibrateExtrinsicsCheckerboard when the scene lacks texture.
%
%   See also: poolPairMatches, relativePoseFromMatches, calibrateIntrinsics,
%             calibrateExtrinsicsCheckerboard, validateCalibration, initSystem

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
% 2. OPEN CAMERAS + LIVE PREVIEW
% -------------------------------------------------------------------------

fprintf('\nOpening cameras...\n');
cams = cell(1, N);
for i = 1:N
    cams{i} = webcam(cfg.camIndices(i));
    cams{i}.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));
    applyCameraSettings(cams{i}, cfg);
end

fprintf('Live preview — aim cameras at a textured, overlapping scene region.\n');
fprintf('Close the window to start the %.0f s pooled capture.\n', ...
        cfg.calExtrinsics.captureSeconds);
fig  = figure('Name', 'Extrinsic calibration — close to start capture', 'NumberTitle', 'off');
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

% -------------------------------------------------------------------------
% 3. POOLED CAPTURE — grab frames for captureSeconds
% -------------------------------------------------------------------------

fprintf('\nCapturing %.0f s of frames for pooled estimation...\n', ...
        cfg.calExtrinsics.captureSeconds);
seq = cell(1, N);
for i = 1:N, seq{i} = {}; end
kf = 0;
t0 = tic;
while toc(t0) < cfg.calExtrinsics.captureSeconds
    kf = kf + 1;
    for i = 1:N
        seq{i}{kf} = snapshot(cams{i});
    end
end
cams = {};
fprintf('Captured %d frames per camera.\n', kf);

% -------------------------------------------------------------------------
% 4. POOLED RELATIVE POSE PER CAMERA PAIR
% -------------------------------------------------------------------------
% Camera 1 is the world reference: R{1} = I, t{1} = 0. For every other camera
% i, pool SURF matches against camera i-1 over all captured frames, fit one
% robust fundamental matrix, and recover the relative pose (premultiply).

R = cell(1, N);  t = cell(1, N);
R{1} = eye(3);   t{1} = zeros(3, 1);
calibDiag = struct('pair', {}, 'info', {}, 'perFrame', {});

for i = 2:N
    ref = i - 1;
    fprintf('\nLocalising camera %d relative to camera %d...\n', i, ref);

    [mRef, mTgt, perFrame] = poolPairMatches(seq{ref}, seq{i}, cfg.calExtrinsics);
    fprintf('  Pooled %d matches across %d frames.\n', size(mRef,1), kf);

    [R_rel, t_rel, info] = relativePoseFromMatches( ...
        mRef, mTgt, intrinsics{ref}, intrinsics{i}, cfg.calExtrinsics);
    fprintf('  %d inliers (%.0f%%), inlier epipolar median %.2f px.\n', ...
            info.nInliers, 100*info.inlierFrac, info.epiMedian);

    if ~isempty(knownBaseline)
        t_rel = t_rel * knownBaseline;     % t_rel is unit
    end

    R{i} = R_rel * R{ref};
    t{i} = R_rel * t{ref} + t_rel;

    calibDiag(end+1) = struct('pair', [ref i], 'info', info, 'perFrame', perFrame); %#ok<SAGROW>
    fprintf('  Camera %d localised. Baseline from cam1: %.3fm\n', i, norm(t{i} - t{1}));
end

% -------------------------------------------------------------------------
% 5. GEOMETRY SUMMARY
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
% 6. ASSEMBLE AND SAVE
% -------------------------------------------------------------------------

multiCamParams.intrinsics   = intrinsics;
multiCamParams.R            = R;
multiCamParams.t            = t;
multiCamParams.capturedAt   = datestr(now);
multiCamParams.calibDiag    = calibDiag;
multiCamParams.nFramesPooled = kf;
multiCamParams.sourceFrames = {seq{1}{1}, seq{2}{1}};   % representative pair

if ~isfolder(fileparts(cfg.calFile))
    mkdir(fileparts(cfg.calFile));
end

save(cfg.calFile, 'multiCamParams');
fprintf('\nCalibration saved to: %s\n', cfg.calFile);
fprintf('Validate with epipolarOnRecording (target <1px) or validateCalibration.\n');


% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function [az, el] = rotationToAzEl(R)
% Camera look direction in world frame = third row of R (world-to-camera premultiply).
az = atan2d(R(3,1), R(3,3));
el = -asind(R(3,2));
end
