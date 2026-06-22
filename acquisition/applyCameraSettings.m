function settled = applyCameraSettings(cam, cfg, mode)
% APPLYCAMERASETTINGS Applies calibrated webcam settings.


if nargin < 3
    mode = 'full';
end

if isfield(cfg, 'resolution') && numel(cfg.resolution) == 2
    trySet(cam, 'Resolution', sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2)));
end

if isfield(cfg, 'cameraControlMode') && strcmpi(cfg.cameraControlMode, 'focusOnly')
    if ~strcmp(mode, 'lock')
        cam.FocusMode = 'manual';
        trySet(cam, 'Focus', cfg.cameraFocus);
    end

    settled = currentCameraState(cam);
    return;
end

prof = profileFor(cam.Name, cfg);

if ~strcmp(mode, 'lock')
    cam.FocusMode = 'manual';
    trySet(cam, 'Focus', cfg.cameraFocus);

    fn   = fieldnames(prof);
    for k = 1:numel(fn)
        if isCameraPropertyField(fn{k})
            trySet(cam, fn{k}, prof.(fn{k}));
        end
    end

    if strcmpi(profileValue(prof, 'ExposureControl', 'auto'), 'manual')
        cam.ExposureMode = 'manual';
        trySet(cam, 'Exposure', profileValue(prof, 'Exposure', tryGet(cam, 'Exposure')));
        trySet(cam, 'Gain',     profileValue(prof, 'Gain',     tryGet(cam, 'Gain')));
    else
        cam.ExposureMode = 'auto';
    end

    if strcmpi(profileValue(prof, 'WhiteBalanceControl', 'auto'), 'manual')
        cam.WhiteBalanceMode = 'manual';
        trySet(cam, 'WhiteBalance', profileValue(prof, 'WhiteBalance', tryGet(cam, 'WhiteBalance')));
    else
        cam.WhiteBalanceMode = 'auto';
    end
end

if strcmp(mode, 'full')
    warnState = warning('off', 'all');
    t0 = tic;
    while toc(t0) < cfg.autoSettleSeconds
        try
            snapshot(cam);
        catch
        end
    end
    warning(warnState);
end

if strcmp(mode, 'full') || strcmp(mode, 'lock')
    if strcmpi(profileValue(prof, 'ExposureControl', 'auto'), 'manual')
        cam.ExposureMode = 'manual';
        trySet(cam, 'Exposure', profileValue(prof, 'Exposure', tryGet(cam, 'Exposure')));
        trySet(cam, 'Gain',     profileValue(prof, 'Gain',     tryGet(cam, 'Gain')));
    else
        autoExposure = tryGet(cam, 'Exposure');
        autoGain     = tryGet(cam, 'Gain');
        cam.ExposureMode = 'manual';
        trySet(cam, 'Exposure', autoExposure);
        trySet(cam, 'Gain',     autoGain);
    end

    autoWhiteBalance = tryGet(cam, 'WhiteBalance');
    cam.WhiteBalanceMode = 'manual';
    trySet(cam, 'WhiteBalance', autoWhiteBalance);
end

settled = currentCameraState(cam);

end

function prof = profileFor(name, cfg)
models = fieldnames(cfg.camProfiles);
for k = 1:numel(models)
    if contains(name, models{k}, 'IgnoreCase', true)
        prof = cfg.camProfiles.(models{k});
        return;
    end
end
prof = struct();
warning('applyCameraSettings:unknownModel', ...
        ['No cfg.camProfiles entry matches camera "%s"; focus + auto-lock ' ...
         'applied, structural settings left at default.'], name);
end

function tf = isCameraPropertyField(fieldName)
profileOnly = {'ExposureControl', 'WhiteBalanceControl', 'Exposure', 'Gain', 'WhiteBalance'};
tf = ~any(strcmp(fieldName, profileOnly));
end

function v = profileValue(prof, fieldName, defaultValue)
if isfield(prof, fieldName)
    v = prof.(fieldName);
else
    v = defaultValue;
end
end

function trySet(cam, prop, val)
try
    cam.(prop) = val;
catch
end
end

function v = tryGet(cam, prop)
try
    v = cam.(prop);
catch
    v = NaN;
end
end

function settled = currentCameraState(cam)
settled = struct('Exposure',     tryGet(cam, 'Exposure'), ...
                 'Gain',         tryGet(cam, 'Gain'), ...
                 'WhiteBalance', tryGet(cam, 'WhiteBalance'), ...
                 'Focus',        tryGet(cam, 'Focus'));
end
