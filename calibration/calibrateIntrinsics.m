% CALIBRATEINTRINSICS  Single-camera intrinsic calibration via auto-capture.
%
%   Script. Edit the USER INPUTS block below, then run with F5.
%
%   Displays a live feed from the selected camera and auto-captures frames
%   when a checkerboard is detected and held steady. Runs estimation once
%   enough frames are collected and saves a cameraParameters object to
%   saveFile. Pass that path to calibrateExtrinsics later.
%
%   If boardSize is empty, the board dimensions are detected automatically
%   from the first frame where a checkerboard is found.
%
%   USER INPUTS
%     camIdx        integer index into webcamlist()
%     squareSizeM   physical side length of one square, in metres
%     boardSize     [rows cols] interior corners, or [] for auto-detect
%                   e.g. [5 7] for a 6x8 square board (corners = squares - 1)
%     saveFile      output path; rename per camera
%     MIN_CAPTURES  minimum captures required before estimation runs
%
%   OUTPUT
%     A .mat file at saveFile containing one variable, intrinsics.
%
%   See also: buildConfig, calibrateExtrinsics, estimateCameraParameters

% =========================================================================
% USER INPUTS — edit these before running
% =========================================================================

disp('Connected cameras:'); disp(webcamlist);

camIdx       = 2;                                  % index into webcamlist()
squareSizeM  = 0.023;                              % physical square size in metres
boardSize    = [7 10];                             % [] = auto-detect on first frame
saveName = 'intrinsics_LG1.mat';   % rename per camera
saveFile = fullfile(fileparts(mfilename('fullpath')), saveName);

cfg = buildConfig();

MIN_CAPTURES  = 25;        % min captures before estimation runs
AUTO_INTERVAL = 2;         % min seconds between auto-captures
STABILITY_THR = 0.5;       % max px drift between frames to count as stable

cam = webcam(camIdx);
cam.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));

% Camera parameters. WARNING: Different camera models return different
% parameters of the webcam() object. Adjust accordingly for script to run!

%MY8077
% cam.FocusMode = 'manual';
% cam.Focus = 0;
% cam.Gain = 0;
% cam.BacklightCompensation = 0;
% cam.Sharpness = 0;
% cam.Gamma = 300;
% cam.Brightness = 0;
% cam.Zoom = 0;
% cam.Pan = 0;
% cam.Tilt = 0;
% cam.Roll = 3;   

%C922 Pro Stream
cam.FocusMode = 'manual';
cam.Focus = 0;                 % 0 = focus at infinity
cam.Gain = 0;
cam.BacklightCompensation = 0;
cam.Sharpness = 0;
cam.Brightness = 128;          % C922 range 0-255; 128 = neutral (NOT 0)
cam.Contrast = 128;
cam.Saturation = 128;
cam.Zoom = 100;                % C922 range 100-500; 100 = no zoom (NOT 0)
cam.Pan = 0;
cam.Tilt = 0;

% Find auto WhiteBalance, then lock
cam.WhiteBalanceMode = 'auto';
pause(3);
cam.WhiteBalanceMode = 'manual';

%Manually set Exposure
cam.ExposureMode = 'manual';
cam.Exposure     = -4;  %fiddle with this

pause(1); 

% Verify intensity is usable before starting capture loop
img_check = rgb2gray(snapshot(cam));
fprintf('Pre-calibration image check — Min:%d  Mean:%.1f  Max:%d\n', ...
    min(img_check(:)), mean(img_check(:)), max(img_check(:)));
if mean(img_check(:)) < 45
    warning('calibrateIntrinsics:darkScene', ...
        'Mean intensity %.1f below target (45). Add light or increase Exposure.', ...
        mean(img_check(:)));
elseif max(img_check(:)) > 130
    warning('calibrateIntrinsics:brightScene', ...
        'Max intensity %d near ceiling (135). Risk of corner clipping.', ...
        max(img_check(:)));
else
    fprintf('Image brightness: good.\n');
end

% =========================================================================

boardConfirmed = ~isempty(boardSize);

H = cfg.resolution(2);
W = cfg.resolution(1);

imagePoints = {};
nCaptured   = 0;
tCalibStart = tic();
lastCapTime = -Inf;        % seconds since tCalibStart
prevPts     = [];

fig = figure('Name', sprintf('Intrinsic calibration — camera %d', camIdx), ...
             'KeyPressFcn', @(~,e) setappdata(gcf,'lastKey',e.Key));
setappdata(fig, 'lastKey', '');

fprintf('\nIntrinsic calibration — camera %d\n', camIdx);
if isempty(boardSize)
    fprintf('Board size not specified — will auto-detect from first frame.\n');
end
fprintf('Hold the checkerboard steady; frames capture automatically.\n');
fprintf('Press Q when done (min %d captures required).\n\n', MIN_CAPTURES);

while ishandle(fig)

    frame    = snapshot(cam);

    % Detect corners without enforcing size — accept whatever the detector finds.
    [pts, detectedSize] = detectCheckerboardPoints(frame);
    boardFound = ~isempty(pts);

    % On first detection, lock in board size and print it for reference.
    if boardFound && ~boardConfirmed
        boardSize      = detectedSize;
        boardConfirmed = true;
        fprintf('Board size auto-detected: [%d %d] interior corners. Starting captures.\n\n', ...
                boardSize(1), boardSize(2));
    end

    % After confirmation, reject frames where a different board size appears.
    if boardFound && boardConfirmed
        boardFound = isequal(detectedSize, boardSize);
    end

    % Check whether the board has moved since the last frame.
    if boardFound && ~isempty(prevPts) && size(pts,1) == size(prevPts,1)
        drift = mean(sqrt(sum((pts - prevPts).^2, 2)));
    else
        drift = Inf;
    end
    isStable = boardFound && drift < STABILITY_THR;

    % Auto-capture when stable and enough time has elapsed.
    nowT       = toc(tCalibStart);
    timePassed = nowT - lastCapTime;
    if isStable && timePassed > AUTO_INTERVAL && all(isfinite(pts(:)))
        nCaptured              = nCaptured + 1;
        imagePoints{nCaptured} = pts;
        lastCapTime            = nowT;
        fprintf('Captured %d/%d\n', nCaptured, MIN_CAPTURES);
    end

    % Draw overlay on live feed.
    display = frame;
    if boardFound && all(isfinite(pts(:)))
        display = insertMarker(frame, pts, 'o', 'Color', 'green', 'Size', 5);
        status  = sprintf('Board found  |  drift %.1fpx  |  captures %d', drift, nCaptured);
    else
        status = sprintf('No board  |  captures %d', nCaptured);
    end
    imshow(display); title(status);
    drawnow;

    prevPts = pts;

    % Exit on Q keypress.
    if strcmp(getappdata(fig,'lastKey'), 'q')
        break;
    end
end

if ishandle(fig), close(fig); end
clear cam;

if nCaptured < MIN_CAPTURES
    error('calibrateIntrinsics:tooFewCaptures', ...
          'Only %d captures collected; need at least %d.', nCaptured, MIN_CAPTURES);
end

fprintf('\nRunning estimation on %d captures...\n', nCaptured);

worldPoints = generateCheckerboardPoints(boardSize, squareSizeM);
imagePoints = cat(3, imagePoints{:});
fprintf('Board size used: [%d %d] interior corners.\n', boardSize(1), boardSize(2));

% Estimate camera parameters from collected image/world point pairs.
intrinsics = estimateCameraParameters( ...
    imagePoints, worldPoints, ...
    'ImageSize',  [H W], ...
    'WorldUnits', 'meters');

reprErrors = intrinsics.ReprojectionErrors;
reprErrors = squeeze(mean(sqrt(sum(reprErrors.^2, 2)), 1));

meanErr = mean(reprErrors);
fprintf('Done. Mean reprojection error: %.3f px\n', meanErr);

if meanErr > 1.0
    warning('calibrateIntrinsics:highError', ...
            'Error %.3fpx exceeds 1px. Recalibrate with more varied poses.', meanErr);
elseif meanErr > 0.5
    fprintf('Warning: error %.3fpx is acceptable but aim for <0.5px.\n', meanErr);
else
    fprintf('Calibration quality: good.\n');
end

% Persist the calibration BEFORE the holdout test, so a good result is never
% lost if the (deprecated) holdout helpers error on a newer MATLAB version.
save(saveFile, 'intrinsics');
fprintf('Saved to %s\n', saveFile);

% =========================================================================
% HOLDOUT VALIDATION
% =========================================================================
fprintf('\nRunning holdout validation...\n');

rng(42);
n = nCaptured;
idx = randperm(n);
n_train = round(0.8 * n);
train_idx = idx(1:n_train);
holdout_idx = idx(n_train+1:end);

% Reconstruct cell arrays for splitting
imagePoints_cell = squeeze(num2cell(imagePoints, [1 2]));   % back to cell of [Nx2] arrays

train_pts = cat(3, imagePoints_cell{train_idx});
holdout_pts_cell = imagePoints_cell(holdout_idx);

% Calibrate on training subset only
intrinsics_train = estimateCameraParameters( ...
    train_pts, worldPoints, ...
    'ImageSize',  [H W], ...
    'WorldUnits', 'meters');

train_err = intrinsics_train.MeanReprojectionError;
fprintf('Training subset error (%.0f%% of data): %.3fpx\n', 80, train_err);

% Evaluate on held-out points
n_holdout = numel(holdout_idx);
holdout_errors = zeros(n_holdout, 1);
valid = false(n_holdout, 1);

for i = 1:n_holdout
    pts_h = holdout_pts_cell{i};
    
    if isempty(pts_h) || any(~isfinite(pts_h(:)))
        continue
    end
    
    valid(i) = true;
    
    % undistort points using training-set intrinsics
    pts_undist = undistortPoints(pts_h, intrinsics_train);
    
    % estimate extrinsics for this held-out view
    [R, t] = extrinsics(pts_undist, worldPoints, intrinsics_train);
    
    % Project world points back into the image. ApplyDistortion MUST be true:
    % pts_h are the raw (distorted) detected corners, so the projection has to
    % carry distortion too. Without it, worldToImage returns ideal pinhole
    % pixels and this "error" just measures lens distortion (large near the
    % frame edges) — which falsely reads as severe overfitting.
    projected = worldToImage(intrinsics_train, R, t, ...
                             [worldPoints, zeros(size(worldPoints,1), 1)], ...
                             'ApplyDistortion', true);

    % per-image reprojection error in raw pixel space (matches MeanReprojectionError)
    diffs = pts_h - projected;
    holdout_errors(i) = mean(sqrt(sum(diffs.^2, 2)));
end

valid_errors = holdout_errors(valid);
holdout_mean = mean(valid_errors);
ratio = holdout_mean / train_err;

fprintf('Held-out error  (%.0f%% of data): %.3fpx\n', 20, holdout_mean);
fprintf('Ratio (held-out / training):      %.2fx\n', ratio);

if ratio < 1.5
    fprintf('Validation: good — calibration generalises well.\n');
elseif ratio < 2.0
    fprintf('Validation: acceptable — mild overfitting, sufficient for application.\n');
elseif ratio < 3.0
    fprintf('Validation: moderate overfitting — recapture with more pose variation.\n');
else
    fprintf('Validation: severe overfitting — calibration unreliable outside training poses.\n');
end

% Plot held-out errors
figure('Name', sprintf('Holdout validation — camera %d', camIdx));
bar(valid_errors);
yline(0.5, 'g--', '0.5px target');
yline(1.0, 'r--', '1px limit');
yline(holdout_mean, 'b--', sprintf('Held-out mean %.3fpx', holdout_mean));
xlabel('Held-out capture index');
ylabel('Reprojection error (px)');
title(sprintf('Camera %d — train %.3fpx | held-out %.3fpx | ratio %.2fx', ...
    camIdx, train_err, holdout_mean, ratio));
% =========================================================================

% Show reprojection error plot for visual inspection.
figure('Name', sprintf('Reprojection errors — camera %d', camIdx));
bar(reprErrors);
xlabel('Capture index'); ylabel('Mean error (px)');
title(sprintf('Camera %d — mean %.3fpx', camIdx, meanErr));
yline(1.0, 'r--', '1px limit');
yline(0.5, 'g--', '0.5px target');
