function saveSession(log, cfg)
% SAVESESSION  Save session log to disk and print summary.
%
%   saveSession(log, cfg)
%
%   INPUTS
%     log — accumulated log struct from logFrame
%     cfg — struct from buildConfig()
%
%   See also: logFrame, initSystem

if ~isfolder(cfg.logDir)
    mkdir(cfg.logDir);
end

filename = fullfile(cfg.logDir, sprintf('session_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(filename, 'log');

nFrames   = numel(log.timestamps);
duration  = log.timestamps(end) - log.timestamps(1);
syncRate  = 100 * sum(log.syncFlags) / nFrames;
meanBlobs = mean(log.nBlobs, 2);   % mean blobs per camera

fprintf('\n--- Session summary ---\n');
fprintf('Frames:      %d  (%.1fs)\n', nFrames, duration);
fprintf('Sync ok:     %.1f%%\n', syncRate);
for i = 1:cfg.N
    fprintf('Cam %d blobs: %.1f mean/frame\n', i, meanBlobs(i));
end
fprintf('Saved to:    %s\n', filename);

end
