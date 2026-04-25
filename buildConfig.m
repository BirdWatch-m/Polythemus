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
cfg.resolution = [1280, 720];  % [width height] px; 720p preferred over 1080p (allows 60fps)
cfg.fps        = 30;           % frames per second; try 60 if cameras support it at cfg.resolution

% --- Ring buffer ---
% Rolling per-camera frame store. New frames overwrite the oldest.
cfg.ringBufLen   = 90;   % total frames stored (~3s at 30fps)
cfg.medianBufLen = 60;   % frames used for temporal median background (~2s)

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

% --- File paths ---
cfg.display     = true;                               % set false to skip live display (~5ms/frame saved)
cfg.logDir      = 'output/';
cfg.calFile     = 'calibration/multiCamParams.mat';   % written by calibrateExtrinsics
cfg.skyMaskFile = 'config/skyMasks.mat';              % written by drawSkyMasks

end
