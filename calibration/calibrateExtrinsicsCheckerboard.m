% CALIBRATEEXTRINSICSCHECKERBOARD  Stereo extrinsic calibration via checkerboard.
%
%   Script. Edit the USER INPUTS block below, then run with F5.
%
%   Opens both cameras side by side. Automatically captures frame pairs
%   when a checkerboard is detected and held steady in BOTH cameras
%   simultaneously. Estimates the relative camera pose from the collected
%   pairs using the existing per-camera intrinsics and saves the result to
%   cfg.calFile in the same format as calibrateExtrinsics.
%
%   Metric scale is derived directly from squareSizeM — no knownBaseline
%   argument required.
%
%   For reliable results: aim for MIN_CAPTURES >= 15 pairs at varied board
%   positions (different distances, tilts, and corners of the frame).
%   The board must be fully visible in BOTH cameras for a pair to capture.
%
%   USER INPUTS
%     intrinsicFiles  {1xN} paths to .mat files from calibrateIntrinsics,
%                     one per camera in cfg.camIndices order
%     squareSizeM     physical side length of one square, in metres
%     boardSize       [rows cols] interior corners, or [] for auto-detect
%                     e.g. [5 7] for a 6x8-square board (corners = squares-1)
%     MIN_CAPTURES    minimum stereo pairs before estimation runs
%
%   OUTPUT
%     cfg.calFile — multiCamParams struct, identical format to calibrateExtrinsics:
%       .intrinsics{i}  cameraParameters object for camera i
%       .R{i}           3x3 rotation matrix for camera i (R{1} = eye(3))
%       .t{i}           3x1 translation vector for camera i (t{1} = zeros)
%       .squareSizeM    square size used (for reference)
%       .capturedAt     timestamp string
%
%   LIMITATIONS
%     Currently supports N = 2 cameras only. The two-camera relative pose
%     is computed per pair via extrinsics() on each camera, then averaged
%     across all pairs using meanrot (rotation) and mean (translation).
%
%   See also: calibrateIntrinsics, calibrateExtrinsics, validateCalibration,
%             buildConfig, initSystem

% =========================================================================
% USER INPUTS — edit these before running
% =========================================================================

clc; close all; clear;

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..')));

cfg = buildConfig();

intrinsicFiles = {'calibration/intrinsics_MY1_720.mat', ...
                  'calibration/intrinsics_LG1_720.mat'};

squareSizeM  = 0.023;    % physical square side length in metres
boardSize    = [7 10];        % [] = auto-detect; or e.g. [5 7]

MIN_CAPTURES  = 15;       % minimum stereo pairs before estimation runs
AUTO_INTERVAL = 3;        % minimum seconds between auto-captures
STABILITY_THR = 1;      % max mean corner drift (px) per camera to count as stable

% =========================================================================

N = cfg.N;
if N ~= 2
    error('calibrateExtrinsicsCheckerboard:unsupportedN', ...
          'This script supports N = 2 cameras only (cfg.N = %d).', N);
end

H = cfg.resolution(2);
W = cfg.resolution(1);

% --- Load intrinsics ---
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

% --- Open cameras + group settle (same approach as initSystem) ---
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

% =========================================================================
% CAPTURE LOOP
% =========================================================================

boardConfirmed = ~isempty(boardSize);

imgPts = cell(N, 0);    % imgPts{i, k} = corners detected in camera i, pair k
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

    % Grab one frame per camera.
    frames = cell(1, N);
    for i = 1:N
        frames{i} = snapshot(cams{i});
    end

    % Detect checkerboard corners in each camera.
    pts   = cell(1, N);
    found = false(1, N);
    detSizes = cell(1, N);
    for i = 1:N
        [pts{i}, detSizes{i}] = detectCheckerboardPoints(frames{i});
        found(i) = ~isempty(pts{i});
    end

    % Auto-detect board size from first successful detection.
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

    % Reject frames where a different board size appears.
    if boardConfirmed
        for i = 1:N
            if found(i) && ~isequal(detSizes{i}, boardSize)
                found(i) = false;
            end
        end
    end

    bothFound = all(found);

    % Stability check — both cameras must be below the drift threshold.
    drifts = Inf(1, N);
    if bothFound
        for i = 1:N
            if ~isempty(prevPts{i}) && size(pts{i}, 1) == size(prevPts{i}, 1)
                drifts(i) = mean(sqrt(sum((pts{i} - prevPts{i}).^2, 2)));
            end
        end
    end
    isStable = bothFound && all(drifts < STABILITY_THR);

    % Auto-capture when stable and enough time has passed.
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

    % Update display.
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

% =========================================================================
% ESTIMATE RELATIVE POSE
% =========================================================================

fprintf('\nEstimating pose from %d pairs...\n', nCaptured);

worldPoints = generateCheckerboardPoints(boardSize, squareSizeM);
worldPts3D  = [worldPoints, zeros(size(worldPoints, 1), 1)];

R_all    = zeros(3, 3, nCaptured);
t_all    = zeros(3, nCaptured);
reprErrs = zeros(1, nCaptured);

for k = 1:nCaptured

    % Board pose in each camera — metric, scale set by squareSizeM.
    % extrinsics() uses postmultiply convention: X_cam_col = R'*X_world_col + t'.
    % R_premultiply = R_postmultiply', so both Rb matrices must be transposed.
    [Rb1, tb1] = extrinsics(imgPts{1,k}, worldPoints, intrinsics{1});
    [Rb2, tb2] = extrinsics(imgPts{2,k}, worldPoints, intrinsics{2});

    % Relative pose: camera 1 -> camera 2 (premultiply, matching pipeline).
    % X_cam2 = R12 * X_cam1 + t12
    R12 = Rb2' * Rb1;
    t12 = tb2' - R12 * tb1';

    R_all(:, :, k) = R12;
    t_all(:, k)    = t12;

    % Reprojection error: project world points into camera 2 and compare
    % to detected corners.
    proj  = worldToImage(intrinsics{2}, Rb2, tb2, worldPts3D, 'ApplyDistortion', true);
    diffs = imgPts{2, k} - proj;
    reprErrs(k) = mean(sqrt(sum(diffs.^2, 2)));

end

fprintf('\nPer-pair reprojection errors (camera 2): ');
fprintf('%.2fpx  ', reprErrs);
fprintf('\nMean: %.3fpx\n', mean(reprErrs));

% Average rotation across pairs via quaternion mean (no toolbox required).
% Build a 4×N matrix of unit quaternions, compute the dominant eigenvector
% of Q*Q', and convert back to a rotation matrix.
Q = zeros(4, nCaptured);
for ki = 1:nCaptured
    q = rotm2quat(R_all(:, :, ki));   % [w x y z], row
    Q(:, ki) = q(:);
end
[V, ~] = eig(Q * Q');
q_mean = V(:, end);             % eigenvector for largest eigenvalue
q_mean = q_mean / norm(q_mean);
R_mean = quat2rotm(q_mean.');
t_mean = mean(t_all, 2);

fprintf('Estimated baseline: %.4f m\n', norm(t_mean));

% =========================================================================
% ASSEMBLE AND SAVE
% =========================================================================

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

% =========================================================================
% GEOMETRY SUMMARY
% =========================================================================

fprintf('\n--- Camera geometry summary (world origin = camera 1) ---\n');
for i = 1:N
    C   = -R{i}' * t{i};           % camera centre in world (cam1) frame
    az  = atan2d(R{i}(3, 1), R{i}(3, 3));   % look direction x/z (third row of R)
    el  = -asind(R{i}(3, 2));               % look direction y (Y-down convention)
    fprintf('Camera %d:  position [%.3f  %.3f  %.3f] m   azimuth %.1f deg   elevation %.1f deg\n', ...
            i, C(1), C(2), C(3), az, el);
end
fprintf('\nVerify baseline against your physical measurement.\n');
fprintf('Run validateCalibration and check reprojection errors (<5px is good).\n');
