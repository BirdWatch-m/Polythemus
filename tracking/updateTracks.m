function [tracks, nextTrackId, counts] = updateTracks(tracks, nextTrackId, measurements, dt, cfg)
% UPDATETRACKS  Advance the 3D Kalman track set by one frame.
%
%   [tracks, nextTrackId, counts] = updateTracks(tracks, nextTrackId, measurements, dt, cfg)
%
%   Maintains a set of constant-velocity Kalman tracks over time. Each frame:
%     1. Predict every track forward by dt.
%     2. Associate this frame's triangulated points to tracks (gated nearest
%        neighbour, resolved one-to-one with matchpairs).
%     3. Matched tracks are Kalman-updated; unmatched tracks coast, then are
%        deleted once they coast longer than cfg.maxCoastFrames.
%     4. Each unmatched point spawns a new tentative track.
%
%   Lifecycle: a track is born 'tentative', becomes 'confirmed' once it has been
%   matched cfg.minConfirmFrames times, and is marked 'coasting' on a missed
%   frame. A tentative track that misses before confirming is dropped at once.
%
%   DESIGN CHOICES (first implementation; see TODO for refinements)
%     - Hand-rolled constant-velocity Kalman (state [px py pz vx vy vz]),
%       measurement = 3D position. Transparent and unit-testable.
%     - Mahalanobis association gate (cfg.trackGate = chi-squared threshold).
%     - Confirmation on CUMULATIVE matched frames, not strictly consecutive.
%
%   INPUTS
%     tracks       — struct array (schema from initSystem: id, state, kf, age,
%                    noDetAge, lastPos); pass an empty such struct for frame 1
%     nextTrackId  — next unused integer track id
%     measurements — [M x 3] valid 3D points this frame (from triangulateGroups)
%     dt           — seconds since the previous update
%     cfg          — buildConfig (uses minConfirmFrames, maxCoastFrames,
%                    kalmanProcNoise, kalmanMeasNoise, trackGate, kalmanInitVelVar)
%
%   OUTPUTS
%     tracks       — updated track struct array
%     nextTrackId  — advanced past any newly spawned ids
%     counts       — struct: measMatched, newSpawned, tentativeDropped, coastedOut
%
%   See also: triangulateGroups, initSystem

nTracks = numel(tracks);
nMeas   = size(measurements, 1);

counts = struct('measMatched', 0, 'newSpawned', 0, 'tentativeDropped', 0, 'coastedOut', 0);

% --- 1. Predict every track forward by dt ---
for t = 1:nTracks
    tracks(t).kf = kfPredict(tracks(t).kf, dt, cfg);
end

% --- 2. Gated nearest-neighbour association (Hungarian) ---
matchedTrack = false(1, nTracks);
assignedMeas = false(1, nMeas);

if nTracks > 0 && nMeas > 0
    C  = inf(nTracks, nMeas);
    R3 = cfg.kalmanMeasNoise * eye(3);
    for t = 1:nTracks
        S    = tracks(t).kf.P(1:3,1:3) + R3;  % innovation covariance (3x3)
        Sinv = S \ eye(3);                      % precompute inverse once per track
        for m = 1:nMeas
            y    = measurements(m,:).' - tracks(t).kf.x(1:3);
            d_sq = y.' * Sinv * y;              % Mahalanobis distance squared
            if d_sq <= cfg.trackGate
                C(t,m) = d_sq;
            end
        end
    end

    A = matchpairs(C, cfg.trackGate);
    counts.measMatched = size(A, 1);
    for k = 1:size(A,1)
        t = A(k,1);  m = A(k,2);
        tracks(t).kf       = kfUpdate(tracks(t).kf, measurements(m,:).', cfg);
        tracks(t).age      = tracks(t).age + 1;
        tracks(t).noDetAge = 0;
        tracks(t).lastPos  = tracks(t).kf.x(1:3).';
        if tracks(t).age >= cfg.minConfirmFrames
            tracks(t).state = 'confirmed';      % confirm, or re-confirm after coasting
        end
        matchedTrack(t) = true;
        assignedMeas(m) = true;
    end
end

% --- 3. Unmatched tracks: coast or delete ---
keep = true(1, nTracks);
for t = 1:nTracks
    if matchedTrack(t), continue; end
    tracks(t).noDetAge = tracks(t).noDetAge + 1;
    tracks(t).lastPos  = tracks(t).kf.x(1:3).';   % report the coasted (predicted) position
    if strcmp(tracks(t).state, 'tentative')
        keep(t) = false;                          % unconfirmed miss -> drop immediately
        counts.tentativeDropped = counts.tentativeDropped + 1;
    else
        tracks(t).state = 'coasting';
        if tracks(t).noDetAge > cfg.maxCoastFrames
            keep(t) = false;                      % coasted too long -> drop
            counts.coastedOut = counts.coastedOut + 1;
        end
    end
end
tracks = tracks(keep);

% --- 4. Spawn a tentative track from each unmatched measurement ---
for m = 1:nMeas
    if assignedMeas(m), continue; end
    tracks(end+1) = spawnTrack(nextTrackId, measurements(m,:), cfg); %#ok<AGROW>
    nextTrackId   = nextTrackId + 1;
    counts.newSpawned = counts.newSpawned + 1;
end

end


% =========================================================================
% LOCAL HELPERS
% =========================================================================

function tr = spawnTrack(id, pos, cfg)
tr.id       = id;
tr.state    = 'tentative';
tr.kf       = kfInit(pos, cfg);
tr.age      = 1;
tr.noDetAge = 0;
tr.lastPos  = pos(:).';
end


function kf = kfInit(pos, cfg)
% Constant-velocity state [px py pz vx vy vz]; zero initial velocity, large
% velocity uncertainty (unknown at birth).
kf.x = [pos(:); 0; 0; 0];
kf.P = diag([cfg.kalmanMeasNoise * [1 1 1], cfg.kalmanInitVelVar * [1 1 1]]);
end


function kf = kfPredict(kf, dt, cfg)
F = [eye(3), dt*eye(3); zeros(3), eye(3)];
G = [0.5*dt^2*eye(3); dt*eye(3)];          % maps random acceleration to state
Q = G * cfg.kalmanProcNoise * G.';
kf.x = F * kf.x;
kf.P = F * kf.P * F.' + Q;
end


function kf = kfUpdate(kf, z, cfg)
H = [eye(3), zeros(3)];
R = cfg.kalmanMeasNoise * eye(3);
y = z - H * kf.x;
S = H * kf.P * H.' + R;
K = (kf.P * H.') / S;
kf.x = kf.x + K * y;
kf.P = (eye(6) - K * H) * kf.P;
end
