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
boardSize    = [];                              % [] = auto-detect on first frame
saveFile     = 'calibration/intrinsics_MY1.mat';   % rename per camera

cfg = buildConfig();

MIN_CAPTURES  = 25;        % min captures before estimation runs
AUTO_INTERVAL = 1.5;       % min seconds between auto-captures
STABILITY_THR = 2.0;       % max px drift between frames to count as stable


% Camera parameters - set here for Myria MY8077 after testing - change if
% different camera

cam = webcam(camIdx);
cam.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));

% Disable all auto-control functions
cam.ExposureMode = 'manual';
cam.WhiteBalanceMode = 'manual';
cam.FocusMode = 'manual';

% Disable processing artifacts
cam.BacklightCompensation = 0;
cam.Sharpness = 0;
cam.Gamma = 100;
cam.Brightness = 0;
cam.Zoom = 0;
cam.Pan = 0;
cam.Tilt = 0;
cam.Roll = 3;

% Calibration values (indoors)
cam.Exposure = -5;
cam.Gain = 64;
cam.WhiteBalance = 4000;   % adjust all 3 of these for ambient conditions
cam.Focus = 100;           % empirically determined sharp-at-1m value

pause(2); 

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

% Show reprojection error plot for visual inspection.
figure('Name', sprintf('Reprojection errors — camera %d', camIdx));
bar(reprErrors);
xlabel('Capture index'); ylabel('Mean error (px)');
title(sprintf('Camera %d — mean %.3fpx', camIdx, meanErr));
yline(1.0, 'r--', '1px limit');
yline(0.5, 'g--', '0.5px target');

save(saveFile, 'intrinsics');
fprintf('Saved to %s\n', saveFile);
