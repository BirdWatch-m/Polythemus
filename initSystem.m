function state = initSystem(cfg)
% INITSYSTEM Opens cameras and initializes runtime state.


H = cfg.resolution(2);
W = cfg.resolution(1);
N = cfg.N;

available = webcamlist();

if numel(cfg.camIndices) ~= N
    error('initSystem:camIndexCount', ...
          'cfg.camIndices has %d entr(y/ies) but cfg.N = %d.', ...
          numel(cfg.camIndices), N);
end
if any(cfg.camIndices < 1) || any(cfg.camIndices > numel(available))
    error('initSystem:camIndexRange', ...
          ['cfg.camIndices = [%s] references a camera outside the %d detected.\n' ...
           'Run webcamlist() and update cfg.camIndices.\nAvailable: %s'], ...
          num2str(cfg.camIndices), numel(available), strjoin(available, ', '));
end

state.cams = cell(1, N);
for i = 1:N
    state.cams{i} = webcam(cfg.camIndices(i));
    state.cams{i}.Resolution = sprintf('%dx%d', W, H);
    applyCameraSettings(state.cams{i}, cfg, 'structural');
end

for i = 1:N
    parts = sscanf(state.cams{i}.Resolution, '%dx%d');
    if abs(parts(1) - W) > 4 || abs(parts(2) - H) > 4
        warning('initSystem:resolutionMismatch', ...
                'Camera %d (webcam %d): requested %dx%d but got %s.', ...
                i, cfg.camIndices(i), W, H, state.cams{i}.Resolution);
    end
end

fprintf('Settling all cameras simultaneously (%.0fs)...\n', cfg.autoSettleSeconds);
warnState = warning('off', 'all');
t0 = tic;
while toc(t0) < cfg.autoSettleSeconds
    for i = 1:N
        try, snapshot(state.cams{i}); catch, end
    end
end
warning(warnState);
for i = 1:N
    settled = applyCameraSettings(state.cams{i}, cfg, 'lock');
    if isfield(cfg, 'cameraControlMode') && strcmpi(cfg.cameraControlMode, 'focusOnly')
        fprintf('  Cam %d focus-only: Exposure = %g, Gain = %g\n', ...
                i, settled.Exposure, settled.Gain);
    else
        fprintf('  Cam %d locked: Exposure = %g, Gain = %g\n', ...
                i, settled.Exposure, settled.Gain);
    end
end

fprintf('Opened %d camera(s).\n', N);

state.ringBuf = cell(1, N);
state.ringIdx = ones(1, N);

for i = 1:N
    state.ringBuf{i} = zeros(H, W, cfg.ringBufLen, 'uint8');
end

fprintf('Ring buffers allocated (%.0f MB per camera).\n', H*W*cfg.ringBufLen/1e6);

state.bgMedian    = cell(1, N);
state.fgDetectors = cell(1, N);

state.bgFramesSinceUpdate = zeros(1, N);

for i = 1:N
    state.bgMedian{i} = zeros(H, W, 'double');

    state.fgDetectors{i} = vision.ForegroundDetector( ...
        'NumGaussians',      3,     ...
        'NumTrainingFrames', 60,    ...
        'LearningRate',      0.005);
end

skyMaskOk = isfile(cfg.skyMaskFile);
if skyMaskOk
    loaded    = load(cfg.skyMaskFile);
    skyMaskOk = numel(loaded.skyMasks) == N;
    if ~skyMaskOk
        warning('initSystem:skyMaskCount', ...
                'Sky mask file has %d mask(s) but cfg.N = %d — using full-frame placeholder.', ...
                numel(loaded.skyMasks), N);
    end
else
    warning('initSystem:noSkyMask', ...
            'Sky mask file not found: %s — using full-frame placeholder.\nRun drawSkyMasks.m before real operation.', ...
            cfg.skyMaskFile);
end

state.skyMask = cell(1, N);
if skyMaskOk
    for i = 1:N
        m = loaded.skyMasks{i};
        if ~isequal(size(m), [H W])
            error('initSystem:skyMaskSize', ...
                  ['Sky mask %d is %dx%d but frames are %dx%d.\n' ...
                   'Re-run drawSkyMasks at the current resolution.'], ...
                  i, size(m,1), size(m,2), H, W);
        end
        state.skyMask{i} = m;
    end
    fprintf('Sky masks loaded.\n');
else
    for i = 1:N
        state.skyMask{i} = true(H, W);
    end
    fprintf('Sky masks: full-frame placeholder active.\n');
end

if ~isfile(cfg.calFile)
    error('initSystem:noCalibration', ...
          'Calibration file not found: %s\nRun calibrateExtrinsics.m first.', cfg.calFile);
end

loaded = load(cfg.calFile);
state.calibration = loaded.multiCamParams;

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

state.calibration = buildFundamentalMatrices(state.calibration, N);

fprintf('Calibration loaded. Fundamental matrices built for %d pair(s).\n', nchoosek(N,2));

state.tracks = struct( ...
    'id',       {}, ...
    'state',    {}, ...
    'kf',       {}, ...
    'age',      {}, ...
    'noDetAge', {}, ...
    'lastPos',  {});

state.nextTrackId = 1;

state.log.timestamps       = [];
state.log.cameraTimestamps = [];
state.log.syncFlags        = [];
state.log.syncMs           = [];
state.log.nBlobs           = [];
state.log.blobCentroids    = {};
state.log.nGroups          = [];
state.log.nPoints          = [];
state.log.nConfirmed       = [];
state.log.meanReprErr      = [];
state.log.trackPositions   = {};

state.tStart   = tic();
state.stopFlag = false;

fprintf('System initialised. Ready.\n');

end
