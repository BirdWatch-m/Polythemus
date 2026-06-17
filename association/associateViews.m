function groups = associateViews(blobs, calibration, cfg)
% ASSOCIATEVIEWS  Group per-camera blobs that view the same 3D point.
%
%   groups = associateViews(blobs, calibration, cfg)
%
%   Matches blobs across cameras using the epipolar constraint: a blob in
%   camera i and a blob in camera j can correspond to the same world point only
%   if the cam-j blob lies within cfg.epiThreshold pixels of the epipolar line
%   predicted by the cam-i blob (via the fundamental matrix F{i,j}).
%
%   For each camera pair, candidate matches are gated by the epipolar distance
%   and resolved one-to-one with a Hungarian assignment (matchpairs). Pairwise
%   matches are then merged into multi-camera groups via connected components,
%   so a point seen in cameras 1 and 2 but not 3 still forms a group (partial
%   observations allowed).
%
%   Convention-agnostic: it consumes whatever F{i,j} it is given and only checks
%   the epipolar relation. Accuracy depends on calibration quality, not on which
%   convention F was built with.
%
%   INPUTS
%     blobs       — {1xN} cell of blob struct arrays (from detectBlobs); each
%                   blob has a .centroid = [u v]
%     calibration — struct with .F{i,j} (3x3 fundamental matrices, i~=j) such
%                   that x_j' * F{i,j} * x_i = 0 for corresponding points
%     cfg         — struct from buildConfig (uses cfg.N, cfg.epiThreshold)
%
%   OUTPUT
%     groups — 1xG struct array, one entry per associated point, with fields:
%       .blobIdx — [1xN] blob index in each camera (0 where not observed)
%       .points  — [Nx2] centroid in each camera (NaN row where not observed)
%       .camIds  — [1xK] cameras that observed this group
%       .nViews  — K = number of observing cameras (>=1)
%     nViews >= 2 is triangulatable; nViews == 1 is a partial observation kept
%     for the tracker.
%
%   LIMITATION
%     With N >= 3, inconsistent pairwise matches could in principle place two
%     blobs from the same camera in one connected component. Exact for N = 2;
%     for N >= 3 the downstream reprojection gate is expected to reject any such
%     inconsistent group.
%
%   See also: detectBlobs, initSystem (buildFundamentalMatrices)

N          = cfg.N;
nBlobs     = cellfun(@numel, blobs);
totalBlobs = sum(nBlobs);

if totalBlobs == 0
    groups = repmat(emptyGroup(N), 1, 0);
    return;
end

% Global node id for (camera i, local blob k) = offset(i) + k.
offset = [0, cumsum(nBlobs(1:end-1))];

% Union-find over all blob nodes; pairwise matches union their two blobs.
parent = 1:totalBlobs;

for i = 1:N
    if nBlobs(i) == 0, continue; end
    for j = i+1:N
        if nBlobs(j) == 0, continue; end

        Fij = calibration.F{i,j};

        % Epipolar-distance cost matrix; Inf where above the gate (forbidden).
        C = inf(nBlobs(i), nBlobs(j));
        for a = 1:nBlobs(i)
            for b = 1:nBlobs(j)
                d = epipolarDistance(blobs{i}(a).centroid, blobs{j}(b).centroid, Fij);
                if d <= cfg.epiThreshold
                    C(a,b) = d;
                end
            end
        end

        % Hungarian one-to-one match; a pair is taken only if cheaper than
        % leaving both unmatched (costUnmatched = epiThreshold).
        M = matchpairs(C, cfg.epiThreshold);
        for m = 1:size(M, 1)
            parent = unite(parent, offset(i) + M(m,1), offset(j) + M(m,2));
        end
    end
end

% Connected components -> groups.
labels = arrayfun(@(x) rootOf(parent, x), 1:totalBlobs);
roots  = unique(labels);

groups = repmat(emptyGroup(N), 1, numel(roots));
for g = 1:numel(roots)
    nodes = find(labels == roots(g));
    for nd = nodes
        i = find(nd > offset, 1, 'last');   % which camera this node belongs to
        k = nd - offset(i);                 % local blob index within that camera
        groups(g).blobIdx(i)  = k;
        groups(g).points(i,:) = blobs{i}(k).centroid;
    end
    groups(g).camIds = find(groups(g).blobIdx > 0);
    groups(g).nViews = numel(groups(g).camIds);
end

end


% =========================================================================
% LOCAL HELPERS
% =========================================================================

function s = emptyGroup(N)
s = struct('blobIdx', zeros(1, N), 'points', nan(N, 2), 'camIds', [], 'nViews', 0);
end


function d = epipolarDistance(x1, x2, F)
% Symmetric point-to-epipolar-line distance (pixels) for a candidate match.
%   x1, x2 — [u v] centroids in cameras i and j
%   F      — F{i,j}; ideally x2' * F * x1 = 0
p1   = [x1(1); x1(2); 1];
p2   = [x2(1); x2(2); 1];
Fp1  = F   * p1;                 % epipolar line of x1 in image j
Ftp2 = F.' * p2;                 % epipolar line of x2 in image i
num  = abs(p2.' * Fp1);          % |x2' F x1|
dj   = num / hypot(Fp1(1),  Fp1(2));    % distance of x2 to its line (image j)
di   = num / hypot(Ftp2(1), Ftp2(2));   % distance of x1 to its line (image i)
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
