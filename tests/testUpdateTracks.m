function testUpdateTracks()
% TESTUPDATETRACKS  Synthetic lifecycle test for updateTracks (no hardware).
%
%   Drives one constant-velocity target through updateTracks frame by frame and
%   checks the track is born, confirmed, follows the trajectory, then coasts and
%   is deleted on schedule once detections stop. Deterministic (no noise).
%
%   Run from the project root:
%       addpath(genpath(pwd)); testUpdateTracks

cfg.minConfirmFrames = 3;
cfg.maxCoastFrames   = 5;
cfg.kalmanProcNoise  = 1.0;
cfg.kalmanMeasNoise  = 0.05;     % m^2
cfg.kalmanInitVelVar = 625;      % (25 m/s)^2
cfg.trackGate        = 3.0;      % m

dt  = 1/30;
p0  = [10; 5; 50];               % m
vel = [8; 0; 2];                 % m/s

tracks      = struct('id',{}, 'state',{}, 'kf',{}, 'age',{}, 'noDetAge',{}, 'lastPos',{});
nextId      = 1;
nDetectFrames = 10;

% --- Phase 1: clean detections of one moving target ---
for f = 1:nDetectFrames
    truePos = (p0 + vel * dt * f).';
    [tracks, nextId] = updateTracks(tracks, nextId, truePos, dt, cfg);
end

assert(numel(tracks) == 1, 'Expected exactly 1 track, got %d.', numel(tracks));
assert(strcmp(tracks(1).state, 'confirmed'), 'Track should be confirmed, is "%s".', tracks(1).state);

posErr = norm(tracks(1).lastPos - (p0 + vel * dt * nDetectFrames).');
assert(posErr < 0.3, 'Tracked position error %.3g m too large.', posErr);

% --- Phase 2: detections stop -> coast for maxCoastFrames, then delete ---
for f = 1:cfg.maxCoastFrames
    [tracks, nextId] = updateTracks(tracks, nextId, zeros(0,3), dt, cfg);
    assert(numel(tracks) == 1, 'Track should still be coasting at miss %d.', f);
    assert(strcmp(tracks(1).state, 'coasting'), 'Track should be coasting at miss %d.', f);
end
[tracks, nextId] = updateTracks(tracks, nextId, zeros(0,3), dt, cfg);
assert(isempty(tracks), 'Track should be deleted after exceeding maxCoastFrames.');

% --- Phase 3: a single spurious detection must NOT confirm ---
[tracks, nextId] = updateTracks(tracks, nextId, [0 0 30], dt, cfg);
assert(numel(tracks) == 1 && strcmp(tracks(1).state, 'tentative'), ...
       'A one-off detection should be a tentative track.');
[tracks, ~] = updateTracks(tracks, nextId, zeros(0,3), dt, cfg);
assert(isempty(tracks), 'A tentative track that misses should be dropped.');

fprintf('testUpdateTracks PASSED: born -> confirmed -> followed -> coasted -> deleted; spurious dropped.\n');

end
