% RECORDSESSION Captures synchronized camera frames and session metadata.

clc; close all; clear;

nSeconds = 0;
outRoot  = 'output/recordings';

cfg = buildConfig();
N = cfg.N;
W = cfg.resolution(1);
H = cfg.resolution(2);

cams = cell(1, N);
for i = 1:N
    cams{i} = webcam(cfg.camIndices(i));
    cams{i}.Resolution = sprintf('%dx%d', W, H);
    applyCameraSettings(cams{i}, cfg, 'structural');
end

fprintf('Settling all cameras simultaneously (%.0fs)...\n', cfg.autoSettleSeconds);
warnState = warning('off', 'all');
t0 = tic;
while toc(t0) < cfg.autoSettleSeconds
    for i = 1:N
        try, snapshot(cams{i}); catch, end
    end
end
warning(warnState);
for i = 1:N
    settled = applyCameraSettings(cams{i}, cfg, 'lock');
    if isfield(cfg, 'cameraControlMode') && strcmpi(cfg.cameraControlMode, 'focusOnly')
        fprintf('  Cam %d focus-only: Exposure = %g, Gain = %g\n', ...
                i, settled.Exposure, settled.Gain);
    else
        fprintf('  Cam %d locked: Exposure = %g, Gain = %g\n', ...
                i, settled.Exposure, settled.Gain);
    end
end

camSettings = cell(1, N);
for i = 1:N
    try, camSettings{i} = get(cams{i}); catch, camSettings{i} = struct('Resolution', cams{i}.Resolution); end
end

sessionDir = fullfile(outRoot, datestr(now, 'yyyymmdd_HHMMSS'));
for i = 1:N
    d = fullfile(sessionDir, sprintf('cam%d', i));
    if ~isfolder(d), mkdir(d); end
end

calibration = [];
calibrationFile = cfg.calFile;
if isfile(cfg.calFile)
    calLoaded = load(cfg.calFile);
    if isfield(calLoaded, 'multiCamParams')
        calibration = calLoaded.multiCamParams;
        copyfile(cfg.calFile, fullfile(sessionDir, 'multiCamParams.mat'));
    else
        warning('recordSession:badCalFormat', ...
                'Calibration file %s does not contain multiCamParams.', cfg.calFile);
    end
else
    warning('recordSession:noCalibration', ...
            'Calibration file not found: %s. Recording will not include an extrinsics snapshot.', ...
            cfg.calFile);
end

estFrames = 10000;
log.timestamps = nan(N, estFrames);
log.syncMs     = nan(1, estFrames);

fig = figure('Name', 'Recording — press Q to stop', 'NumberTitle', 'off', ...
             'KeyPressFcn', @(~,e) setappdata(gcf, 'lastKey', e.Key));
setappdata(fig, 'lastKey', '');

fprintf('Recording %d camera(s) at %dx%d to:\n  %s\nPress Q to stop.\n', N, W, H, sessionDir);
tStart = tic;
frameCount = 0;

while ishandle(fig)

    [frames, timestamps] = acquireFrames(cams, tStart, cfg);
    frameCount = frameCount + 1;

    for i = 1:N
        fname = fullfile(sessionDir, sprintf('cam%d', i), sprintf('frame_%06d.tif', frameCount));
        imwrite(rgb2gray(frames{i}), fname, 'Compression', 'none');
    end

    log.timestamps(:, frameCount) = timestamps(:);
    log.syncMs(frameCount)        = (max(timestamps) - min(timestamps)) * 1000;

    if mod(frameCount, 30) == 0
        fprintf('  %d frames | t=%.1fs | sync ~%.1fms | ~%.1f fps\n', ...
                frameCount, toc(tStart), log.syncMs(frameCount), frameCount / toc(tStart));
    end

    if nSeconds > 0 && toc(tStart) >= nSeconds, break; end
    if strcmp(getappdata(fig, 'lastKey'), 'q'), break; end
end

if ishandle(fig), close(fig); end
cams = {};

log.timestamps = log.timestamps(:, 1:frameCount);
log.syncMs     = log.syncMs(1:frameCount);

session.cfg         = cfg;
session.camSettings = camSettings;
session.calibration = calibration;
session.calibrationFile = calibrationFile;
session.nFrames     = frameCount;
session.log         = log;
save(fullfile(sessionDir, 'session.mat'), 'session');

fprintf('Done. %d frames/camera saved to %s\n', frameCount, sessionDir);
