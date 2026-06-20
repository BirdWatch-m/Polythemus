function settled = applyCameraSettings(cam, cfg, mode)
% APPLYCAMERASETTINGS  Lock focus + structural settings, settle then lock auto modes.
%
%   settled = applyCameraSettings(cam, cfg)
%   settled = applyCameraSettings(cam, cfg, mode)
%
%   Configures an already-opened webcam for capture. MUST be called at every
%   camera-open site — otherwise the camera runs on autofocus/auto-exposure,
%   which (a) changes the effective focal length so it no longer matches the
%   intrinsic calibration, and (b) drifts frame-to-frame, breaking background
%   subtraction.
%
%   MODE values:
%     'full'       (default) — structural + settle + lock. Used by single-camera
%                  callers (calibrateExtrinsics, drawSkyMasks, testSingleCamera).
%     'structural' — structural settings + set auto mode, no settle, no lock.
%                  Used by initSystem before the group settle loop.
%     'lock'       — lock current auto values only (no structural, no settle).
%                  Used by initSystem after the group settle loop.
%
%   A future exposure/gain optimisation (autoTune — low priority, see TODO.md)
%   belongs right here, refining the locked exposure by metering over the sky ROI.
%
%   INPUTS
%     cam  — an opened webcam object (Resolution already set)
%     cfg  — buildConfig (uses cameraFocus, autoSettleSeconds, camProfiles)
%     mode — string, default 'full' (see above)
%
%   OUTPUT
%     settled — struct of the current values (Exposure, WhiteBalance, Focus).
%               In 'structural' mode these reflect the auto state and are not
%               yet stable.
%
%   See also: buildConfig, initSystem, acquireFrames, calibrateIntrinsics

if nargin < 3
    mode = 'full';
end

prof = profileFor(cam.Name, cfg);

if ~strcmp(mode, 'lock')
    % --- 1. Manual focus (must match calibration) ---
    cam.FocusMode = 'manual';
    trySet(cam, 'Focus', cfg.cameraFocus);

    % --- 2. Per-model structural locks (skip any property the camera lacks) ---
    fn   = fieldnames(prof);
    for k = 1:numel(fn)
        if isCameraPropertyField(fn{k})
            trySet(cam, fn{k}, prof.(fn{k}));
        end
    end

    % --- 3. Exposure/WB policy ---
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
    % Snapshot during settle so auto algorithms advance (also warms up the
    % camera). Switching to manual afterwards holds the converged values.
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

settled = struct('Exposure',     tryGet(cam, 'Exposure'), ...
                 'Gain',         tryGet(cam, 'Gain'), ...
                 'WhiteBalance', tryGet(cam, 'WhiteBalance'), ...
                 'Focus',        tryGet(cam, 'Focus'));

end


% =========================================================================
% LOCAL HELPERS
% =========================================================================

function prof = profileFor(name, cfg)
% Pick the cfg.camProfiles entry whose key is a substring of the camera name.
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
% Profile-only control keys are interpreted here, not sent to the webcam.
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
% Set a property only if this model exposes it (models differ).
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
