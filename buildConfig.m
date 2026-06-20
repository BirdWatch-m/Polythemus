function cfg = buildConfig()
% BUILDCONFIG  Create the system configuration struct.
%
%   cfg = buildConfig() returns a struct containing every tunable parameter
%   in the system. No magic numbers appear anywhere else in the codebase.
%
%   OUTPUT
%     cfg — struct (fields documented inline below)
%
%   TUNING GUIDE
%     Run the system, then inspect the session log in cfg.logDir.
%     The log records reprojection errors, sync flags, and blob/track counts
%     per frame. Use those numbers to guide threshold adjustments.
%
%   See also: initSystem, main

% --- Camera setup ---
cfg.N          = 2;            % number of cameras; only change this line to add/remove cameras
% Physical webcamlist() index for each logical camera (camera 1 = world origin).
% Run webcamlist() and VERIFY these each session — the order is OS-driven and can
% change between reboots/replugs. The laptop built-in is usually index 1, so a
% rig of two external cameras is typically [2 3]. Must have cfg.N entries.
cfg.camIndices = [2 3];
cfg.resolution = [1280, 720]; % [width height] px;
cfg.fps        = 30;           % frames per second; try 60 if cameras support it at cfg.resolution

% --- Camera capture settings (applied by applyCameraSettings at every open) ---
% Focus is MANUAL and shared with calibration so operation matches the intrinsics
% (focus changes the effective focal length). Set cameraControlMode to
% 'focusOnly' to leave the webcam driver in charge of exposure, white balance,
% and image processing while still forcing the calibrated focus.
cfg.cameraFocus       = 0;    % manual focus (0 = infinity); MUST match calibrateIntrinsics
cfg.cameraControlMode = 'focusOnly';
cfg.autoSettleSeconds = 5;    % let auto-exposure/WB converge before locking them
cfg.camProfiles.MY8077 = struct('ExposureControl','auto', ...
    'BacklightCompensation',0, 'Sharpness',0, 'Gamma',300, ...
    'Brightness',0, 'Contrast',64, 'Saturation',128, ...
    'Zoom',0, 'Pan',0, 'Tilt',0, 'Roll',3);
cfg.camProfiles.C922   = struct('ExposureControl','auto', ...
    'BacklightCompensation',0, 'Sharpness',128, ...
    'Brightness',128, 'Contrast',128, 'Saturation',128, 'Zoom',100, 'Pan',0, 'Tilt',0);

% --- Ring buffer ---
% Rolling per-camera frame store. New frames overwrite the oldest.
cfg.ringBufLen   = 90;   % total frames stored (~3s at 30fps)
cfg.medianBufLen = 60;   % frames used for temporal median background (~2s)

% --- Background model ---
cfg.bgUpdateInterval  = 15;  % recompute the median background every N frames (per camera)
cfg.medianFgThreshold = 35;  % |frame - median| above this (0-255) marks a pixel foreground
cfg.bgMedianStride    = 3;   % subsample the ring buffer by this stride when taking the median (speed)
cfg.useGMM            = true; % AND the GMM model with the median model; false = median-only (faster)

% --- Morphology (foreground mask cleanup before blob extraction) ---
% WARNING: imopen erodes a morphKernelRadius-pixel border, so it can erase
% small distant targets (a 3x3px bird vs minBlobArea=9). Evaluate whether this
% costs real detections at range before trusting it.
cfg.useMorphology     = true; % open+close the mask; false = skip entirely
cfg.morphKernelRadius = 2;    % disk radius (px) for open/close
cfg.morphStrel        = strel('disk', cfg.morphKernelRadius);  % built once; rebuild if radius changes

% --- Detection thresholds ---
% Epipolar: max distance (px) from predicted epipolar line to accept a cross-camera match.
cfg.epiThreshold  = 3.0;   % increase to 5.0 if valid birds are being rejected

% Reprojection: max error (px) between triangulated point and original detection.
cfg.reprThreshold = 3.0;   % increase if few points survive triangulation

% Blob area: filters out sensor noise (too small) and clouds/buildings (too large).
cfg.minBlobArea = 9;       % px^2 — ~3x3px, smallest detectable bird at max range
cfg.maxBlobArea = 2000;    % px^2 — adjust upward for large close-range seagulls

% Aspect ratio: major/minor axis of blob ellipse. Rejects compact noise and diffuse clouds.
cfg.maxAspect = 6.0;

% Sync: flag frame sets where any camera pair differs by more than this (seconds).
cfg.maxSyncError = 0.033;  % one frame at 30fps; does not discard frames, only logs flag

% --- Tracking ---
cfg.maxCoastFrames   = 5;   % consecutive missed frames before a track is deleted
cfg.minConfirmFrames = 3;   % consecutive matched frames before a track is reported

% Kalman process noise: how much random acceleration to assume (m/s^2)^2.
cfg.kalmanProcNoise = 1.0;   % increase for erratic species (swallows); decrease for pigeons

% Kalman measurement noise: expected triangulation uncertainty (m^2).
cfg.kalmanMeasNoise = 2.0;   % ~1.5m sigma at 100m with 8m baseline; tune after validation

% Track association gate: max distance (m) from a predicted track to a new point
% to accept the match. Tune to expected per-frame motion + triangulation error.
cfg.trackGate = 5.0;

% Initial velocity variance for a new track (m/s)^2 — large, since velocity is
% unknown at birth.
cfg.kalmanInitVelVar = 625;   % (25 m/s)^2, covers the fastest expected birds

% --- Extrinsic calibration (SURF, multi-frame pooled) ---
% calibrateExtrinsics pools SURF correspondences over a short capture and fits one
% robust fundamental matrix. A single frame pair is multi-modal on repetitive
% scenes (façade mismatches form competing RANSAC consensuses); pooling many
% frames makes the true geometry the dominant consensus. See diagnostics/
% montecarloExtrinsicsPooled for the frame-count vs stability evidence.
cfg.calExtrinsics.captureSeconds   = 5;      % seconds of frames to pool per run
cfg.calExtrinsics.surfMetricThresh = 300;    % detectSURFFeatures MetricThreshold
cfg.calExtrinsics.matchThreshold   = 50;     % matchFeatures MatchThreshold
cfg.calExtrinsics.maxRatio         = 0.7;    % matchFeatures MaxRatio
cfg.calExtrinsics.ransacNumTrials  = 8000;   % estimateFundamentalMatrix trials
cfg.calExtrinsics.ransacDistance   = 1.0;    % inlier epipolar distance (px)
cfg.calExtrinsics.ransacConfidence = 99.99;  % RANSAC confidence (%)
cfg.calExtrinsics.minPooledInliers = 100;    % abort below this many pooled matches
cfg.calExtrinsics.maxPoseMatches   = 20000;  % cap inliers fed to relativeCameraPose: its pose
                                             % decomposition is O(n^2) in memory (tens of thousands
                                             % of matches OOM); a few thousand fully fix the pose.
                                             % F is still estimated on ALL pooled matches.
cfg.calExtrinsics.fixCam2Coplanar  = true;   % true: force cam2 forward (depth) offset to 0.
                                             % The forward offset is unobservable from a distant
                                             % scene (low parallax) and drifts run-to-run; for a
                                             % level side-by-side rig it is ~0 by construction.
cfg.calExtrinsics.fixCam2Level     = true;   % true: force cam2 vertical (level) offset to 0.
                                             % Y IS observable (epipolar tilt) so it is already
                                             % tight (~+/-7cm); this asserts equal camera height.
                                             % Set false if mounts can differ in height by >few cm.
                                             % Both flags apply to BOTH extrinsic methods (SURF and
                                             % checkerboard); together they pin cam2 to [B,0,0].
                                             % See diagnostics/extrinsicsStability.

% --- File paths ---
cfg.display     = true;                               % set false to skip live display (~5ms/frame saved)
cfg.logDir      = 'output/';
cfg.calFile     = 'calibration/multiCamParams.mat';   % written by calibrateExtrinsics
cfg.skyMaskFile = 'config/skyMasks.mat';              % written by drawSkyMasks

end
