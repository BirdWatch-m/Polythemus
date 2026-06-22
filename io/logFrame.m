function log = logFrame(log, blobs, timestamps, syncOk, cfg)
% LOGFRAME Appends one frame to the session log.


if ~isfield(log, 'timestamps'),       log.timestamps = [];       end
if ~isfield(log, 'cameraTimestamps'), log.cameraTimestamps = []; end
if ~isfield(log, 'syncFlags'),        log.syncFlags = [];        end
if ~isfield(log, 'syncMs'),           log.syncMs = [];           end
if ~isfield(log, 'nBlobs'),           log.nBlobs = [];           end
if ~isfield(log, 'blobCentroids'),    log.blobCentroids = {};    end

log.timestamps(end+1)          = mean(timestamps);
log.cameraTimestamps(:, end+1) = timestamps(:);
log.syncFlags(end+1)           = syncOk;
log.syncMs(end+1)              = (max(timestamps) - min(timestamps)) * 1000;

counts = cellfun(@numel, blobs);
log.nBlobs(:,end+1) = counts(:);

log.blobCentroids{end+1} = cellfun(@(b) vertcat(b.centroid), blobs, 'UniformOutput', false);

end
