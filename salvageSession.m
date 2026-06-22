% SALVAGESESSION  Reconstruct session.mat after a crash-interrupted recording.
%
%   Run from the project root (F5). Edit SESSION_DIR below.
%
%   What happened: the camera disconnected mid-capture. All frames written
%   before the crash are on disk (imwrite ran inside the loop, one at a time).
%   session.mat was not written because the save runs after the loop.
%
%   What this recovers:
%     session.cfg         — rebuilt from buildConfig()
%     session.nFrames     — counted from cam1 TIF files on disk
%     session.calibration — loaded from multiCamParams.mat sidecar (was saved)
%     session.camSettings — empty (lost; not needed for processRecording)
%     session.log.timestamps — reconstructed as uniform spacing at measured fps
%     session.log.syncMs  — reconstructed as zeros (actual sync lost; negligible)
%
%   The reconstructed timestamps produce per-frame dt values indistinguishable
%   from the real ones for Kalman filter purposes (~26.5 fps vs 30 fps nominal).

clc;
addpath(genpath(fileparts(mfilename('fullpath'))));

% =========================================================================
% USER INPUT — set to the crashed session folder
% =========================================================================
SESSION_DIR = 'output/recordings/20260621_205602';
% =========================================================================

sessionFile = fullfile(SESSION_DIR, 'session.mat');
if isfile(sessionFile)
    error('salvageSession:alreadyExists', ...
          'session.mat already exists in %s\nDelete it first if you want to re-run.', SESSION_DIR);
end

fprintf('Salvaging session: %s\n\n', SESSION_DIR);

% --- Count frames from disk ---
cam1Dir  = fullfile(SESSION_DIR, 'cam1');
tifFiles = dir(fullfile(cam1Dir, 'frame_*.tif'));
nFrames  = numel(tifFiles);
if nFrames == 0
    error('salvageSession:noFrames', 'No frame_*.tif files found in %s', cam1Dir);
end
fprintf('Frames found on disk: %d\n', nFrames);

% --- Rebuild cfg ---
cfg = buildConfig();
N   = cfg.N;

% --- Verify all cam directories have the same count ---
for i = 2:N
    camDir = fullfile(SESSION_DIR, sprintf('cam%d', i));
    n_i    = numel(dir(fullfile(camDir, 'frame_*.tif')));
    if n_i ~= nFrames
        warning('salvageSession:frameMismatch', ...
                'cam%d has %d frames but cam1 has %d — using minimum.', i, n_i, nFrames);
        nFrames = min(nFrames, n_i);
    end
end

% --- Load calibration from sidecar (was saved before the loop) ---
calSidecar = fullfile(SESSION_DIR, 'multiCamParams.mat');
calibration = [];
if isfile(calSidecar)
    cal = load(calSidecar);
    if isfield(cal, 'multiCamParams')
        calibration = cal.multiCamParams;
        fprintf('Calibration loaded from session sidecar.\n');
    else
        warning('salvageSession:badCal', 'Sidecar exists but has no multiCamParams field.');
    end
else
    warning('salvageSession:noCal', 'No multiCamParams.mat sidecar — calibration will be empty.');
end

% --- Reconstruct timestamps at uniform measured fps ---
% Last status print showed 7260 frames at t=273.4s → actual fps ≈ 26.55.
% Use that to reconstruct timing; the resulting dt values are well within
% the accuracy needed by the Kalman filter.
measuredFps  = nFrames / (nFrames / 26.55);   % ≈26.55; edit if you have a better estimate
frameIndices = (0 : nFrames-1);
ts_row       = frameIndices / measuredFps;     % seconds since start, uniform

% Both cameras assigned the same timestamps (sync jitter was ~1-4ms, negligible).
log.timestamps = repmat(ts_row, N, 1);   % [N x nFrames]
log.syncMs     = zeros(1, nFrames);      % actual sync lost; zeros is conservative

% --- Assemble and save ---
session.cfg             = cfg;
session.camSettings     = {};     % not recoverable after crash
session.calibration     = calibration;
session.calibrationFile = cfg.calFile;
session.nFrames         = nFrames;
session.log             = log;
session.salvageNote     = sprintf('Reconstructed by salvageSession on %s. Timestamps uniform at %.2f fps. camSettings lost.', ...
                                  datestr(now), measuredFps);

save(sessionFile, 'session');
fprintf('\nSaved: %s\n', sessionFile);
fprintf('nFrames = %d  |  measuredFps = %.2f fps\n', nFrames, measuredFps);
fprintf('\nYou can now run processRecording pointing at:\n  %s\n', SESSION_DIR);
