function settled = applyCameraSettings(cam, cfg)
% APPLYCAMERASETTINGS  Lock focus + structural settings, settle then lock auto modes.
%
%   settled = applyCameraSettings(cam, cfg)
%
%   Configures an already-opened webcam for capture. MUST be called at every
%   camera-open site — otherwise the camera runs on autofocus/auto-exposure,
%   which (a) changes the effective focal length so it no longer matches the
%   intrinsic calibration, and (b) drifts frame-to-frame, breaking background
%   subtraction.
%
%   What it does:
%     1. Manual focus at cfg.cameraFocus — the SAME value calibrateIntrinsics
%        uses, so operation focus == calibration focus (single-sourced in cfg).
%     2. Per-model structural locks (gain, sharpness, gamma, etc.) from
%        cfg.camProfiles, selected by a substring match on cam.Name.
%     3. Lets auto-exposure and auto-white-balance converge to the current scene
%        (snapshotting during the settle, which also warms the camera up), then
%        switches both to manual so they HOLD at the converged values.
%
%   A future exposure/gain optimisation (autoTune — low priority, see TODO.md)
%   belongs right here, refining the locked exposure by metering over the sky ROI.
%
%   INPUTS
%     cam — an opened webcam object (Resolution already set)
%     cfg — buildConfig (uses cameraFocus, autoSettleSeconds, camProfiles)
%
%   OUTPUT
%     settled — struct of the locked values (Exposure, WhiteBalance, Focus), for
%               logging / reproducibility
%
%   See also: buildConfig, initSystem, acquireFrames, calibrateIntrinsics

% --- 1. Manual focus (must match calibration) ---
cam.FocusMode = 'manual';
trySet(cam, 'Focus', cfg.cameraFocus);

% --- 2. Per-model structural locks (skip any property the camera lacks) ---
prof = profileFor(cam.Name, cfg);
fn   = fieldnames(prof);
for k = 1:numel(fn)
    trySet(cam, fn{k}, prof.(fn{k}));
end

% --- 3. Auto-converge exposure + WB to the scene, then lock them ---
% Auto algorithms advance as frames stream, so snapshot during the settle
% (this also serves as the camera warmup). Switching to manual holds the value.
cam.ExposureMode     = 'auto';
cam.WhiteBalanceMode = 'auto';
warnState = warning('off', 'all');
t0 = tic;
while toc(t0) < cfg.autoSettleSeconds
    try, snapshot(cam); catch, end
end
warning(warnState);
cam.ExposureMode     = 'manual';   % holds at the converged exposure
cam.WhiteBalanceMode = 'manual';   % holds at the converged white balance

settled = struct('Exposure', tryGet(cam, 'Exposure'), ...
                 'WhiteBalance', tryGet(cam, 'WhiteBalance'), ...
                 'Focus', tryGet(cam, 'Focus'));

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
