function state = initSystem(cfg)
% INITSYSTEM  Open cameras and allocate all system state.
%
%   state = initSystem(cfg) opens all N cameras, allocates the ring buffer
%   and background models, loads calibration and sky masks, and returns all
%   runtime state in a single struct for use by the main loop.
%
%   INPUT
%     cfg — struct from buildConfig()
%
%   OUTPUT
%     state — struct with fields:
%       .cams         {1xN} cell of webcam objects
%       .ringBuf      {1xN} cell of H x W x ringBufLen uint8 arrays
%       .ringIdx      [1xN] current write position per ring buffer
%       .bgMedian     {1xN} cell of H x W double (median background image)
%       .fgDetectors  {1xN} cell of vision.ForegroundDetector objects
%       .skyMask      {1xN} cell of H x W logical arrays
%       .calibration  struct — camera params + fundamental matrices (see below)
%       .tracks       struct array — empty, schema defined here
%       .nextTrackId  integer — increments each time a new track is spawned
%       .log          struct — empty session log
%       .stopFlag     logical — set true to exit the main loop
%       .tStart       uint64 — tic() reference for session timestamps
%
%   CALIBRATION STRUCT (state.calibration)
%     Loaded from cfg.calFile (written by calibrateExtrinsics), then
%     augmented with fundamental matrices computed here.
%       .intrinsics{i}  cameraIntrinsics object for camera i
%       .R{i}           3x3 rotation matrix for camera i (R{1} = eye(3))
%       .t{i}           3x1 translation vector for camera i (t{1} = zeros)
%       .F{i,j}         3x3 fundamental matrix, computed on load (not stored)
%
%   ERRORS
%     Stops with an informative message if fewer than N cameras are found,
%     or if either required .mat file (calFile, skyMaskFile) is missing.
%
%   See also: buildConfig, acquireFrames, main

H = cfg.resolution(2);
W = cfg.resolution(1);
N = cfg.N;

% -------------------------------------------------------------------------
% 1. OPEN CAMERAS
% -------------------------------------------------------------------------

available = webcamlist();

if numel(available) < N
    error('initSystem:notEnoughCameras', ...
          'Config requests %d cameras but only %d detected.\nAvailable: %s', ...
          N, numel(available), strjoin(available, ', '));
end

state.cams = cell(1, N);
for i = 1:N
    state.cams{i} = webcam(i);
    state.cams{i}.Resolution = sprintf('%dx%d', W, H);
end

% Warn if camera silently rounded to a different resolution.
for i = 1:N
    parts = sscanf(state.cams{i}.Resolution, '%dx%d');
    if abs(parts(1) - W) > 4 || abs(parts(2) - H) > 4
        warning('initSystem:resolutionMismatch', ...
                'Camera %d: requested %dx%d but got %s.', ...
                i, W, H, state.cams{i}.Resolution);
    end
end

fprintf('Opened %d camera(s).\n', N);

% -------------------------------------------------------------------------
% 2. ALLOCATE RING BUFFERS
% -------------------------------------------------------------------------
% Shape: H x W x ringBufLen per camera. uint8 to minimise memory (~83MB/cam at 720p).

state.ringBuf = cell(1, N);
state.ringIdx = ones(1, N);

for i = 1:N
    state.ringBuf{i} = zeros(H, W, cfg.ringBufLen, 'uint8');
end

fprintf('Ring buffers allocated (%.0f MB per camera).\n', H*W*cfg.ringBufLen/1e6);

% -------------------------------------------------------------------------
% 3. INITIALISE BACKGROUND MODELS
% -------------------------------------------------------------------------

state.bgMedian    = cell(1, N);
state.fgDetectors = cell(1, N);

for i = 1:N
    state.bgMedian{i} = zeros(H, W, 'double');   % populated once ring buffer fills

    % GMM detector: 3 Gaussians per pixel, slow learning rate for outdoor stability.
    state.fgDetectors{i} = vision.ForegroundDetector( ...
        'NumGaussians',      3,     ...
        'NumTrainingFrames', 60,    ...
        'LearningRate',      0.005);
end

% -------------------------------------------------------------------------
% 4. LOAD SKY MASKS
% -------------------------------------------------------------------------

if ~isfile(cfg.skyMaskFile)
    error('initSystem:noSkyMask', ...
          'Sky mask file not found: %s\nRun drawSkyMasks.m first.', cfg.skyMaskFile);
end

loaded = load(cfg.skyMaskFile);

if numel(loaded.skyMasks) ~= N
    error('initSystem:skyMaskCount', ...
          'Sky mask file has %d mask(s) but cfg.N = %d.', numel(loaded.skyMasks), N);
end

state.skyMask = cell(1, N);
for i = 1:N
    state.skyMask{i} = loaded.skyMasks{i};
end

fprintf('Sky masks loaded.\n');

% -------------------------------------------------------------------------
% 5. LOAD CALIBRATION AND BUILD FUNDAMENTAL MATRICES
% -------------------------------------------------------------------------

if ~isfile(cfg.calFile)
    error('initSystem:noCalibration', ...
          'Calibration file not found: %s\nRun calibrateExtrinsics.m first.', cfg.calFile);
end

loaded = load(cfg.calFile);
state.calibration = loaded.multiCamParams;

% Validate expected fields from calibrateExtrinsics output format.
if ~isfield(state.calibration, 'intrinsics') || ...
   ~isfield(state.calibration, 'R')          || ...
   ~isfield(state.calibration, 't')
    error('initSystem:badCalFormat', ...
          ['Calibration file is missing required fields.\n' ...
           'Expected .intrinsics, .R, .t — re-run calibrateExtrinsics.m.']);
end

if numel(state.calibration.intrinsics) ~= N
    error('initSystem:calCameraMismatch', ...
          'Calibration contains %d camera(s) but cfg.N = %d.', ...
          numel(state.calibration.intrinsics), N);
end

% Compute fundamental matrices for all camera pairs and attach to struct.
state.calibration = buildFundamentalMatrices(state.calibration, N);

fprintf('Calibration loaded. Fundamental matrices built for %d pair(s).\n', nchoosek(N,2));

% -------------------------------------------------------------------------
% 6. INITIALISE EMPTY TRACK LIST
% -------------------------------------------------------------------------
% Empty struct array with schema matching fields written by initTrack.m.

state.tracks = struct( ...
    'id',       {}, ...
    'state',    {}, ...   % 'tentative' | 'confirmed' | 'coasting'
    'kf',       {}, ...   % trackingKF object
    'age',      {}, ...   % total frames alive
    'noDetAge', {}, ...   % consecutive frames without detection
    'lastPos',  {});      % [x y z] last confirmed position (m)

state.nextTrackId = 1;

% -------------------------------------------------------------------------
% 7. INITIALISE SESSION LOG
% -------------------------------------------------------------------------

state.log.timestamps     = [];   % [1 x nFrames] seconds since tStart
state.log.syncFlags      = [];   % [1 x nFrames] logical
state.log.nBlobs         = [];   % [N x nFrames] blob count per camera
state.log.nGroups        = [];   % [1 x nFrames] association groups formed
state.log.nConfirmed     = [];   % [1 x nFrames] confirmed tracks
state.log.meanReprErr    = [];   % [1 x nFrames] mean reprojection error (px)
state.log.trackPositions = {};   % {nFrames x 1} cell of [nTracks x 3] arrays

% -------------------------------------------------------------------------
% 8. REMAINING STATE
% -------------------------------------------------------------------------

state.tStart   = tic();
state.stopFlag = false;

fprintf('System initialised. Ready.\n');

end


% =========================================================================
% LOCAL HELPER
% =========================================================================

function calibration = buildFundamentalMatrices(calibration, N)
% Computes F{i,j} for all camera pairs from .intrinsics, .R, .t fields.
% F{i,j} maps a point in camera i to its epipolar line in camera j.
% Uses the relation: F = inv(K_j)' * T_skew * R_rel * inv(K_i)

calibration.F = cell(N, N);

for i = 1:N
    for j = 1:N
        if i == j, continue; end

        % Intrinsic matrices from cameraIntrinsics objects.
        % IntrinsicMatrix is stored column-major in MATLAB; transpose to get K.
        K_i = calibration.intrinsics{i}.IntrinsicMatrix';
        K_j = calibration.intrinsics{j}.IntrinsicMatrix';

        % Relative rotation and translation: world <- i, then i <- j.
        R_rel = calibration.R{j} * calibration.R{i}';
        t_rel = calibration.t{j} - R_rel * calibration.t{i};

        % Skew-symmetric matrix of t_rel, used to encode the cross product.
        tx = t_rel(1); ty = t_rel(2); tz = t_rel(3);
        T_skew = [ 0,  -tz,  ty;
                   tz,  0,  -tx;
                  -ty,  tx,   0];

        E = T_skew * R_rel;                          % essential matrix
        calibration.F{i,j} = inv(K_j)' * E * inv(K_i);  % fundamental matrix
    end
end

end
