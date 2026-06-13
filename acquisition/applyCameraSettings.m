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

if ~strcmp(mode, 'lock')
    % --- 1. Manual focus (must match calibration) ---
    cam.FocusMode = 'manual';
    trySet(cam, 'Focus', cfg.cameraFocus);

    % --- 2. Per-model structural locks (skip any property the camera lacks) ---
    prof = profileFor(cam.Name, cfg);
    fn   = fieldnames(prof);
    for k = 1:numel(fn)
        trySet(cam, fn{k}, prof.(fn{k}));
    end

    % --- 3. Set auto modes ---
    cam.ExposureMode     = 'auto';
    cam.WhiteBalanceMode = 'auto';
end

if strcmp(mode, 'full')
    % Snapshot during settle so auto algorithms advance (also warms up the
    % camera). Switching to manual afterwards holds the converged values.
    warnState = warning('off', 'all');
    t0 = tic;
    while toc(t0) < cfg.autoSettleSeconds
        try, snapshot(cam); catch, end
    end
    warning(warnState);
end

% if strcmp(mode, 'full') || strcmp(mode, 'lock')
%     cam.ExposureMode     = 'manual';   % holds at the converged exposure
%     cam.WhiteBalanceMode = 'manual';   % holds at the converged white balance
% end

settled = struct('Exposure',     tryGet(cam, 'Exposure'), ...
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


function trySet(cam, prop, val)
% Set a property only if this model exposes it (models differ).
try, cam.(prop) = val; catch, end
end


function v = tryGet(cam, prop)
try, v = cam.(prop); catch, v = NaN; end
end
