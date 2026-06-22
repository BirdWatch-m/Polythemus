function [points, counts] = triangulateGroups(groups, calibration, cfg)
% TRIANGULATEGROUPS Triangulates associated blobs into 3D points.


N = cfg.N;

counts = struct('skippedFewViews', 0, 'failedRepr', 0, 'failedNegZ', 0, ...
                'failedMaxRange', 0, 'valid', 0);

P = cell(1, N);
for i = 1:N
    K    = calibration.intrinsics{i}.IntrinsicMatrix.';
    P{i} = K * [calibration.R{i}, calibration.t{i}(:)];
end

points = repmat(emptyPoint(), 1, 0);

for g = 1:numel(groups)
    cams = groups(g).camIds;
    if numel(cams) < 2
        counts.skippedFewViews = counts.skippedFewViews + 1;
        continue;
    end

    obs = zeros(numel(cams), 2);
    Ps  = cell(1, numel(cams));
    for c = 1:numel(cams)
        i        = cams(c);
        obs(c,:) = undistortPoints(groups(g).points(i,:), calibration.intrinsics{i});
        Ps{c}    = P{i};
    end

    [X, reprojErr, reprojErrs, reprojPts] = triangulateDLT(obs, Ps);

    okRepr  = reprojErr <= cfg.reprThreshold;
    okNegZ  = X(3) > 0;
    okRange = norm(X) <= cfg.maxRange;

    if ~okRepr,  counts.failedRepr     = counts.failedRepr     + 1; end
    if ~okNegZ,  counts.failedNegZ     = counts.failedNegZ     + 1; end
    if ~okRange, counts.failedMaxRange = counts.failedMaxRange + 1; end

    p                           = emptyPoint(N);
    p.position                  = X(:).';
    p.reprojErr                 = reprojErr;
    p.reprojErrByCam(cams)      = reprojErrs;
    p.reprojectedPoints(cams,:) = reprojPts;
    p.camIds                    = cams;
    p.groupIdx                  = g;
    p.valid                     = okRepr && okNegZ && okRange;

    if p.valid, counts.valid = counts.valid + 1; end
    points(end+1) = p;
end

end

function p = emptyPoint(N)
if nargin < 1
    N = 0;
end
p = struct('position', [NaN NaN NaN], 'reprojErr', NaN, ...
           'reprojErrByCam', nan(1, N), 'reprojectedPoints', nan(N, 2), ...
           'camIds', [], 'groupIdx', 0, 'valid', false);
end

function [X, reprojErr, errs, reprojPts] = triangulateDLT(pts, Ps)
K = numel(Ps);

A = zeros(2*K, 4);
for c = 1:K
    Pc = Ps{c};
    u  = pts(c,1); v = pts(c,2);
    A(2*c-1, :) = u * Pc(3,:) - Pc(1,:);
    A(2*c,   :) = v * Pc(3,:) - Pc(2,:);
end

[~, ~, V] = svd(A, 0);
Xh = V(:, end);
X  = Xh(1:3) / Xh(4);

errs = zeros(1, K);
reprojPts = zeros(K, 2);
for c = 1:K
    x       = Ps{c} * [X; 1];
    x       = x(1:2) / x(3);
    reprojPts(c,:) = x(:).';
    errs(c) = hypot(x(1) - pts(c,1), x(2) - pts(c,2));
end
reprojErr = mean(errs);
end
