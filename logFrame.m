function log = logFrame(log, blobs, timestamps, syncOk, cfg)
% LOGFRAME  Append one frame's data to the session log.
%
%   log = logFrame(log, blobs, timestamps, syncOk, cfg)
%
%   INPUTS
%     log        — current log struct from initSystem or previous logFrame
%     blobs      — {1xN} cell of blob struct arrays from detectBlobs
%     timestamps — [1xN] timestamps from acquireFrames
%     syncOk     — logical from syncCheck
%     cfg        — struct from buildConfig()
%
%   OUTPUT
%     log — updated log struct with one new entry appended
%
%   See also: saveSession, initSystem, replaySession

% Mean timestamp across cameras as the frame time.
log.timestamps(end+1) = mean(timestamps);
log.syncFlags(end+1)  = syncOk;

% Blob count per camera this frame.
counts = cellfun(@numel, blobs);   % [1xN] blob counts
log.nBlobs(:,end+1) = counts(:);

% Store blob centroid
log.blobCentroids{end+1} = cellfun(@(b) vertcat(b.centroid), blobs, 'UniformOutput', false);

end
