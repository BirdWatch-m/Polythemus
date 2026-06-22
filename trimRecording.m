% TRIMRECORDING  Copy a recording folder trimmed to a frame range.

baseDir = 'D:\HomeWork\Licenta\Program\output\recordings';
srcDir  = fullfile(baseDir, '20260622_132103');

jobs = {
    '20260622_132103_f2200_end', 2200, inf;
};

for ji = 1:size(jobs, 1)
    dstName = jobs{ji, 1};
    i1      = jobs{ji, 2};
    i2raw   = jobs{ji, 3};
    dstDir  = fullfile(baseDir, dstName);

% --- Load session ---
loaded  = load(fullfile(srcDir, 'session.mat'));
session = loaded.session;
cfg     = session.cfg;
N       = cfg.N;

i2 = min(i2raw, session.nFrames);
frameIndices = i1:i2;
nNew = numel(frameIndices);
fprintf('\n[%s] frames %d to %d  → %d frames\n', dstName, i1, i2, nNew);

% --- Create output directories ---
if isfolder(dstDir)
    warning('trimRecording:exists', 'Destination already exists, files will be overwritten: %s', dstDir);
else
    mkdir(dstDir);
end
for c = 1:N
    camDst = fullfile(dstDir, sprintf('cam%d', c));
    if ~isfolder(camDst), mkdir(camDst); end
end

% --- Copy and renumber frames ---
for c = 1:N
    fprintf('Copying cam%d frames...\n', c);
    for j = 1:nNew
        k   = frameIndices(j);
        src = fullfile(srcDir, sprintf('cam%d', c), sprintf('frame_%06d.tif', k));
        dst = fullfile(dstDir, sprintf('cam%d', c), sprintf('frame_%06d.tif', j));
        copyfile(src, dst);
        if mod(j, 500) == 0
            fprintf('  cam%d: %d / %d\n', c, j, nNew);
        end
    end
    fprintf('  cam%d done.\n', c);
end

% --- Build trimmed session.mat ---
sl = session.log;
% Slice every vector/matrix field that has one column per frame.
logFields = fieldnames(sl);
for lfi = 1:numel(logFields)
    f = logFields{lfi};
    v = sl.(f);
    if isnumeric(v) || islogical(v)
        if size(v, 2) == session.nFrames
            sl.(f) = v(:, frameIndices);
        elseif numel(v) == session.nFrames && isrow(v)
            sl.(f) = v(frameIndices);
        end
    elseif iscell(v) && numel(v) == session.nFrames
        sl.(f) = v(frameIndices);
    end
end

newSession         = session;
newSession.log     = sl;
newSession.nFrames = nNew;

log     = newSession.log;   % keep legacy top-level variable
session = newSession;
save(fullfile(dstDir, 'session.mat'), 'log', 'session');
fprintf('Saved session.mat\n');

% --- Copy multiCamParams.mat ---
srcCal = fullfile(srcDir, 'multiCamParams.mat');
if isfile(srcCal)
    copyfile(srcCal, fullfile(dstDir, 'multiCamParams.mat'));
    fprintf('Copied multiCamParams.mat\n');
end

fprintf('Done: %s\n', dstDir);
end % jobs loop
