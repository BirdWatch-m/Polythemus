function ok = syncCheck(timestamps, cfg)
% SYNCCHECK  Check whether all cameras are within acceptable time sync.
%
%   ok = syncCheck(timestamps, cfg)
%
%   Computes the maximum pairwise timestamp difference across all cameras.
%   Returns false and prints a warning if it exceeds cfg.maxSyncError.
%   Does NOT discard the frame — caller continues regardless; the flag
%   is recorded in the session log for post-session filtering.
%
%   INPUTS
%     timestamps — [1xN] seconds since tStart, one per camera
%     cfg        — struct from buildConfig()
%
%   OUTPUT
%     ok — logical; true if all cameras are within cfg.maxSyncError
%
%   See also: acquireFrames, logFrame

maxDiff = max(timestamps) - min(timestamps);
ok      = maxDiff <= cfg.maxSyncError;

if ~ok
    fprintf('Sync warning: max timestamp gap %.1fms (limit %.1fms)\n', ...
            maxDiff*1000, cfg.maxSyncError*1000);
end

end
