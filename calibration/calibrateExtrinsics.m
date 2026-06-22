% CALIBRATEEXTRINSICS Calibrates inter-camera extrinsics from pooled feature matches.

clc; close all; clear;

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..')));

cfg = buildConfig();

intrinsicFiles = {'calibration/intrinsics_MY1_720.mat', ...
                  'calibration/intrinsics_LG1_720.mat'};

knownBaseline = 1.2;

N = cfg.N;

if numel(intrinsicFiles) ~= N
    error('calibrateExtrinsics:fileMismatch', ...
          'intrinsicFiles has %d entries but cfg.N = %d.', ...
          numel(intrinsicFiles), N);
end

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
        t_rel = t_rel * knownBaseline;
    end

    R{i} = R_rel * R{ref};
    t{i} = R_rel * t{ref} + t_rel;

    calibDiag(end+1) = struct('pair', [ref i], 'info', info, 'perFrame', perFrame);
    fprintf('  Camera %d localised. Baseline from cam1: %.3fm\n', i, norm(t{i} - t{1}));
end

fprintf('\n--- Camera geometry summary (world origin = camera 1) ---\n');
for i = 1:N
    C = -R{i}' * t{i};
    [az, el] = rotationToAzEl(R{i});
    fprintf('Camera %d:  position [%.3f  %.3f  %.3f] m   azimuth %.1f deg   elevation %.1f deg\n', ...
            i, C(1), C(2), C(3), az, el);
end
fprintf('\nVerify baselines against your physical telemeter measurements.\n');

multiCamParams.intrinsics   = intrinsics;
multiCamParams.R            = R;
multiCamParams.t            = t;
multiCamParams.capturedAt   = datestr(now);
multiCamParams.calibDiag    = calibDiag;
multiCamParams.nFramesPooled = kf;
multiCamParams.sourceFrames = {seq{1}{1}, seq{2}{1}};

if ~isfolder(fileparts(cfg.calFile))
    mkdir(fileparts(cfg.calFile));
end

save(cfg.calFile, 'multiCamParams');
fprintf('\nCalibration saved to: %s\n', cfg.calFile);
fprintf('Validate with epipolarOnRecording (target <1px) or validateCalibration.\n');

function [az, el] = rotationToAzEl(R)
az = atan2d(R(3,1), R(3,3));
el = -asind(R(3,2));
end
