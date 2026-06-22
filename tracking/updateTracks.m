function [tracks, nextTrackId, counts] = updateTracks(tracks, nextTrackId, measurements, dt, cfg)
% UPDATETRACKS Updates 3D Kalman tracks from current measurements.


nTracks = numel(tracks);
nMeas   = size(measurements, 1);

counts = struct('measMatched', 0, 'newSpawned', 0, 'tentativeDropped', 0, 'coastedOut', 0);

for t = 1:nTracks
    tracks(t).kf = kfPredict(tracks(t).kf, dt, cfg);
end

matchedTrack = false(1, nTracks);
assignedMeas = false(1, nMeas);

if nTracks > 0 && nMeas > 0
    C  = inf(nTracks, nMeas);
    R3 = cfg.kalmanMeasNoise * eye(3);
    for t = 1:nTracks
        S    = tracks(t).kf.P(1:3,1:3) + R3;
        Sinv = S \ eye(3);
        for m = 1:nMeas
            y    = measurements(m,:).' - tracks(t).kf.x(1:3);
            d_sq = y.' * Sinv * y;
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
            tracks(t).state = 'confirmed';
        end
        matchedTrack(t) = true;
        assignedMeas(m) = true;
    end
end

keep = true(1, nTracks);
for t = 1:nTracks
    if matchedTrack(t), continue; end
    tracks(t).noDetAge = tracks(t).noDetAge + 1;
    tracks(t).lastPos  = tracks(t).kf.x(1:3).';
    if strcmp(tracks(t).state, 'tentative')
        keep(t) = false;
        counts.tentativeDropped = counts.tentativeDropped + 1;
    else
        tracks(t).state = 'coasting';
        if tracks(t).noDetAge > cfg.maxCoastFrames
            keep(t) = false;
            counts.coastedOut = counts.coastedOut + 1;
        end
    end
end
tracks = tracks(keep);

for m = 1:nMeas
    if assignedMeas(m), continue; end
    tracks(end+1) = spawnTrack(nextTrackId, measurements(m,:), cfg);
    nextTrackId   = nextTrackId + 1;
    counts.newSpawned = counts.newSpawned + 1;
end

end

function tr = spawnTrack(id, pos, cfg)
tr.id       = id;
tr.state    = 'tentative';
tr.kf       = kfInit(pos, cfg);
tr.age      = 1;
tr.noDetAge = 0;
tr.lastPos  = pos(:).';
end

function kf = kfInit(pos, cfg)
kf.x = [pos(:); 0; 0; 0];
kf.P = diag([cfg.kalmanMeasNoise * [1 1 1], cfg.kalmanInitVelVar * [1 1 1]]);
end

function kf = kfPredict(kf, dt, cfg)
F = [eye(3), dt*eye(3); zeros(3), eye(3)];
G = [0.5*dt^2*eye(3); dt*eye(3)];
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
