% CALIBRATEEXTRINSICSCHECKERBOARD Calibrates stereo extrinsics from checkerboard pairs.

clc; close all; clear;

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..')));

cfg = buildConfig();

intrinsicFiles = {'calibration/intrinsics_MY1_720.mat', ...
                  'calibration/intrinsics_LG1_720.mat'};

squareSizeM  = 0.031;
boardSize    = [7 10];

MIN_CAPTURES  = 15;
AUTO_INTERVAL = 3;
STABILITY_THR = 1;

N = cfg.N;
if N ~= 2
    error('calibrateExtrinsicsCheckerboard:unsupportedN', ...
          'This script supports N = 2 cameras only (cfg.N = %d).', N);
end

H = cfg.resolution(2);
W = cfg.resolution(1);

fprintf('Loading intrinsics...\n');
intrinsics = cell(1, N);
for i = 1:N
    if ~isfile(intrinsicFiles{i})
        error('calibrateExtrinsicsCheckerboard:missingFile', ...
              'Intrinsics file not found: %s', intrinsicFiles{i});
    end
    loaded = load(intrinsicFiles{i});
    fn = fieldnames(loaded);
    intrinsics{i} = loaded.(fn{1});
    fprintf('  Camera %d: %.1fpx focal length.\n', i, mean(intrinsics{i}.FocalLength));
end

fprintf('\nOpening cameras...\n');
cams = cell(1, N);
for i = 1:N
    cams{i} = webcam(cfg.camIndices(i));
    cams{i}.Resolution = sprintf('%dx%d', W, H);
    applyCameraSettings(cams{i}, cfg, 'structural');
end

fprintf('Settling cameras simultaneously (%.0fs)...\n', cfg.autoSettleSeconds);
warnState = warning('off', 'all');
t0 = tic;
while toc(t0) < cfg.autoSettleSeconds
    for i = 1:N
        try, snapshot(cams{i}); catch, end
    end
end
warning(warnState);
for i = 1:N
    settled = applyCameraSettings(cams{i}, cfg, 'lock');
    if isfield(cfg, 'cameraControlMode') && strcmpi(cfg.cameraControlMode, 'focusOnly')
        fprintf('  Cam %d focus-only: Exposure = %g, Gain = %g\n', ...
                i, settled.Exposure, settled.Gain);
    else
        fprintf('  Cam %d locked: Exposure = %g, Gain = %g\n', ...
                i, settled.Exposure, settled.Gain);
    end
end

boardConfirmed = ~isempty(boardSize);

imgPts = cell(N, 0);
nCaptured   = 0;
tStart      = tic();
lastCapTime = -Inf;
prevPts     = cell(1, N);

fig = figure('Name', 'Stereo extrinsic calibration — press Q when done', ...
             'NumberTitle', 'off', ...
             'KeyPressFcn', @(~,e) setappdata(gcf, 'lastKey', e.Key));
setappdata(fig, 'lastKey', '');

fprintf('\nHold the checkerboard steady in view of BOTH cameras.\n');
fprintf('Move to a new position between auto-captures.\n');
fprintf('Vary distance, tilt, and frame position for best results.\n');
fprintf('Press Q when done (min %d pairs required).\n\n', MIN_CAPTURES);

while ishandle(fig)

    frames = cell(1, N);
    for i = 1:N
        frames{i} = snapshot(cams{i});
    end

    pts   = cell(1, N);
    found = false(1, N);
    detSizes = cell(1, N);
    for i = 1:N
        [pts{i}, detSizes{i}] = detectCheckerboardPoints(frames{i});
        found(i) = ~isempty(pts{i});
    end

    if any(found) && ~boardConfirmed
        for i = 1:N
            if found(i)
                boardSize      = detSizes{i};
                boardConfirmed = true;
                fprintf('Board size auto-detected: [%d %d] interior corners.\n\n', ...
                        boardSize(1), boardSize(2));
                break;
            end
        end
    end

    if boardConfirmed
        for i = 1:N
            if found(i) && ~isequal(detSizes{i}, boardSize)
                found(i) = false;
            end
        end
    end

    bothFound = all(found);

    drifts = Inf(1, N);
    if bothFound
        for i = 1:N
            if ~isempty(prevPts{i}) && size(pts{i}, 1) == size(prevPts{i}, 1)
                drifts(i) = mean(sqrt(sum((pts{i} - prevPts{i}).^2, 2)));
            end
        end
    end
    isStable = bothFound && all(drifts < STABILITY_THR);

    nowT = toc(tStart);
    if isStable && (nowT - lastCapTime) > AUTO_INTERVAL && ...
       all(isfinite(pts{1}(:))) && all(isfinite(pts{2}(:)))
        nCaptured = nCaptured + 1;
        for i = 1:N
            imgPts{i, nCaptured} = pts{i};
        end
        lastCapTime = nowT;
        fprintf('Captured pair %d/%d\n', nCaptured, MIN_CAPTURES);
    end

    try
        for i = 1:N
            subplot(1, N, i);
            disp = frames{i};
            if found(i) && all(isfinite(pts{i}(:)))
                disp = insertMarker(frames{i}, pts{i}, 'o', 'Color', 'green', 'Size', 5);
                camStr = sprintf('Board  drift %.1fpx', drifts(i));
            else
                camStr = 'No board';
            end
            imshow(disp);
            title(sprintf('Cam %d — %s', i, camStr));
        end
        if isStable
            sgtitle(sprintf('Pairs: %d/%d  [STABLE — hold still]', nCaptured, MIN_CAPTURES));
        else
            sgtitle(sprintf('Pairs: %d/%d', nCaptured, MIN_CAPTURES));
        end
        drawnow;
    catch
        break;
    end

    for i = 1:N
        prevPts{i} = pts{i};
    end

    if strcmp(getappdata(fig, 'lastKey'), 'q')
        break;
    end
end

if ishandle(fig), close(fig); end
cams = {};

if nCaptured < MIN_CAPTURES
    error('calibrateExtrinsicsCheckerboard:tooFewPairs', ...
          'Only %d pairs collected; need at least %d. Re-run and capture more.', ...
          nCaptured, MIN_CAPTURES);
end

fprintf('\nEstimating pose from %d pairs...\n', nCaptured);

worldPoints = generateCheckerboardPoints(boardSize, squareSizeM);
worldPts3D  = [worldPoints, zeros(size(worldPoints, 1), 1)];

R_all    = zeros(3, 3, nCaptured);
t_all    = zeros(3, nCaptured);
reprErrs = zeros(1, nCaptured);

for k = 1:nCaptured

    [Rb1, tb1] = extrinsics(imgPts{1,k}, worldPoints, intrinsics{1});
    [Rb2, tb2] = extrinsics(imgPts{2,k}, worldPoints, intrinsics{2});

    R12 = Rb2' * Rb1;
    t12 = tb2' - R12 * tb1';

    R_all(:, :, k) = R12;
    t_all(:, k)    = t12;

    proj  = worldToImage(intrinsics{2}, Rb2, tb2, worldPts3D, 'ApplyDistortion', true);
    diffs = imgPts{2, k} - proj;
    reprErrs(k) = mean(sqrt(sum(diffs.^2, 2)));

end

fprintf('\nPer-pair reprojection errors (camera 2): ');
fprintf('%.2fpx  ', reprErrs);
fprintf('\nMean: %.3fpx\n', mean(reprErrs));

Q = zeros(4, nCaptured);
for ki = 1:nCaptured
    q = rotm2quat(R_all(:, :, ki));
    Q(:, ki) = q(:);
end
[V, ~] = eig(Q * Q');
q_mean = V(:, end);
q_mean = q_mean / norm(q_mean);
R_mean = quat2rotm(q_mean.');
t_mean = mean(t_all, 2);

fprintf('Estimated baseline: %.4f m\n', norm(t_mean));

zeroLevel = cfg.calExtrinsics.fixCam2Level;
zeroDepth = cfg.calExtrinsics.fixCam2Coplanar;
if zeroLevel || zeroDepth
    C_before = -R_mean.' * t_mean;
    [R_mean, t_mean] = constrainCam2Centre(R_mean, t_mean, zeroLevel, zeroDepth);
    C_after = -R_mean.' * t_mean;
    fprintf('Cam2 centre constrained (level=%d depth=%d): [%.3f %.3f %.3f] -> [%.3f %.3f %.3f] m\n', ...
            zeroLevel, zeroDepth, C_before, C_after);
end

R = {eye(3),      R_mean};
t = {zeros(3, 1), t_mean};

multiCamParams.intrinsics   = intrinsics;
multiCamParams.R            = R;
multiCamParams.t            = t;
multiCamParams.squareSizeM  = squareSizeM;
multiCamParams.capturedAt   = datestr(now);

if ~isempty(fileparts(cfg.calFile)) && ~isfolder(fileparts(cfg.calFile))
    mkdir(fileparts(cfg.calFile));
end

save(cfg.calFile, 'multiCamParams');
fprintf('\nCalibration saved to: %s\n', cfg.calFile);
fprintf('Run validateCalibration to verify triangulation accuracy.\n');

fprintf('\n--- Camera geometry summary (world origin = camera 1) ---\n');
for i = 1:N
    C   = -R{i}' * t{i};
    az  = atan2d(R{i}(3, 1), R{i}(3, 3));
    el  = -asind(R{i}(3, 2));
    fprintf('Camera %d:  position [%.3f  %.3f  %.3f] m   azimuth %.1f deg   elevation %.1f deg\n', ...
            i, C(1), C(2), C(3), az, el);
end
fprintf('\nVerify baseline against your physical measurement.\n');
fprintf('Run validateCalibration and check reprojection errors (<5px is good).\n');
