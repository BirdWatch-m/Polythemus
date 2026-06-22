function [groups, counts] = associateViews(blobs, calibration, cfg)
% ASSOCIATEVIEWS Associates per-camera blobs using epipolar geometry.


N          = cfg.N;
nBlobs     = cellfun(@numel, blobs);
totalBlobs = sum(nBlobs);

counts = struct('candidatePairs', 0, 'rejectedEpi', 0, 'matchedPairs', 0, ...
                'singletonGroups', 0, 'multiViewGroups', 0);

if totalBlobs == 0
    groups = repmat(emptyGroup(N), 1, 0);
    return;
end

offset = [0, cumsum(nBlobs(1:end-1))];

parent = 1:totalBlobs;

for i = 1:N
    if nBlobs(i) == 0, continue; end
    for j = i+1:N
        if nBlobs(j) == 0, continue; end

        Fij = calibration.F{i,j};

        C = inf(nBlobs(i), nBlobs(j));
        for a = 1:nBlobs(i)
            for b = 1:nBlobs(j)
                d = epipolarDistance(blobs{i}(a).centroid, blobs{j}(b).centroid, Fij);
                if d <= cfg.epiThreshold
                    C(a,b) = d;
                end
            end
        end

        nCandidates = nBlobs(i) * nBlobs(j);
        nFinite     = sum(isfinite(C(:)));
        counts.candidatePairs = counts.candidatePairs + nCandidates;
        counts.rejectedEpi    = counts.rejectedEpi    + (nCandidates - nFinite);

        M = matchpairs(C, cfg.epiThreshold);
        counts.matchedPairs = counts.matchedPairs + size(M, 1);
        for m = 1:size(M, 1)
            parent = unite(parent, offset(i) + M(m,1), offset(j) + M(m,2));
        end
    end
end

labels = arrayfun(@(x) rootOf(parent, x), 1:totalBlobs);
roots  = unique(labels);

groups = repmat(emptyGroup(N), 1, numel(roots));
for g = 1:numel(roots)
    nodes = find(labels == roots(g));
    for nd = nodes
        i = find(nd > offset, 1, 'last');
        k = nd - offset(i);
        groups(g).blobIdx(i)  = k;
        groups(g).points(i,:) = blobs{i}(k).centroid;
    end
    groups(g).camIds = find(groups(g).blobIdx > 0);
    groups(g).nViews = numel(groups(g).camIds);
end

if ~isempty(groups)
    nViews_all = [groups.nViews];
    counts.singletonGroups = sum(nViews_all == 1);
    counts.multiViewGroups = sum(nViews_all >= 2);
end

end

function s = emptyGroup(N)
s = struct('blobIdx', zeros(1, N), 'points', nan(N, 2), 'camIds', [], 'nViews', 0);
end

function d = epipolarDistance(x1, x2, F)
p1   = [x1(1); x1(2); 1];
p2   = [x2(1); x2(2); 1];
Fp1  = F   * p1;
Ftp2 = F.' * p2;
num  = abs(p2.' * Fp1);
dj   = num / hypot(Fp1(1),  Fp1(2));
di   = num / hypot(Ftp2(1), Ftp2(2));
d    = 0.5 * (di + dj);
end

function r = rootOf(parent, x)
while parent(x) ~= x
    x = parent(x);
end
r = x;
end

function parent = unite(parent, a, b)
ra = rootOf(parent, a);
rb = rootOf(parent, b);
if ra ~= rb
    parent(rb) = ra;
end
end
