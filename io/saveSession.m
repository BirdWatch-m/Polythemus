function saveSession(log, cfg, state)
% SAVESESSION  Save session log to disk and print summary.
%
%   saveSession(log, cfg)
%   saveSession(log, cfg, state)
%
%   INPUTS
%     log   - accumulated log struct from logFrame
%     cfg   - struct from buildConfig()
%     state - optional runtime state from initSystem/main. When provided,
%             saveSession also persists calibration, camera settings, and
%             final tracks.
%
%   See also: logFrame, initSystem

if nargin < 3
    state = struct();
end

if ~isfolder(cfg.logDir)
    mkdir(cfg.logDir);
end

stamp    = datestr(now, 'yyyymmdd_HHMMSS');
filename = fullfile(cfg.logDir, sprintf('session_%s.mat', stamp));

nFrames   = numel(log.timestamps);
duration  = log.timestamps(end) - log.timestamps(1);
syncRate  = 100 * sum(log.syncFlags) / nFrames;
meanBlobs = mean(log.nBlobs, 2);   % mean blobs per camera

session.savedAt = datestr(now);
session.cfg     = cfg;
session.nFrames = nFrames;
session.log     = log;

if isfield(cfg, 'calFile')
    session.calibrationFile = cfg.calFile;
else
    session.calibrationFile = '';
end

if isfield(state, 'calibration')
    session.calibration = state.calibration;
end

if isfield(state, 'tracks')
    session.finalTracks = state.tracks;
end

if isfield(state, 'cams') && ~isempty(state.cams)
    session.camSettings = cell(1, numel(state.cams));
    for i = 1:numel(state.cams)
        try
            session.camSettings{i} = get(state.cams{i});
        catch
            session.camSettings{i} = struct();
        end
    end
end

if isfield(session, 'calibration')
    multiCamParams = session.calibration;
    calSnapshotFile = fullfile(cfg.logDir, sprintf('session_%s_multiCamParams.mat', stamp));
    save(calSnapshotFile, 'multiCamParams');
    session.calibrationSnapshotFile = calSnapshotFile;
end

% Keep the legacy top-level "log" variable for old analysis scripts.
save(filename, 'log', 'session');

fprintf('\n--- Session summary ---\n');
fprintf('Frames:      %d  (%.1fs)\n', nFrames, duration);
fprintf('Sync ok:     %.1f%%\n', syncRate);
for i = 1:cfg.N
    fprintf('Cam %d blobs: %.1f mean/frame\n', i, meanBlobs(i));
end
if isfield(session, 'calibrationSnapshotFile')
    fprintf('Calibration: %s\n', session.calibrationSnapshotFile);
end
fprintf('Saved to:    %s\n', filename);

end
