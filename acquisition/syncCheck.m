function ok = syncCheck(timestamps, cfg)
% SYNCCHECK Checks inter-camera timestamp spread.


maxDiff = max(timestamps) - min(timestamps);
ok      = maxDiff <= cfg.maxSyncError;

if ~ok
    fprintf('Sync warning: max timestamp gap %.1fms (limit %.1fms)\n', ...
            maxDiff*1000, cfg.maxSyncError*1000);
end

end
