% CALIBRATEINTRINSICS Calibrates one camera intrinsic model.

disp('Connected cameras:'); disp(webcamlist);

camIdx       = 2;
squareSizeM  = 0.023;
boardSize    = [7 10];
saveName = 'intrinsics_MY1_720.mat';
saveFile = fullfile(fileparts(mfilename('fullpath')), saveName);

cfg = buildConfig();

MIN_CAPTURES  = 25;
AUTO_INTERVAL = 3;
STABILITY_THR = 0.5;

cam = webcam(camIdx);
cam.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));

applyCameraSettings(cam, cfg);

cam.Exposure = -1;
pause(1);

previewFig = figure('Name', 'Exposure preview — press any key to START, Ctrl-C to abort', ...
                    'KeyPressFcn', @(~,e) setappdata(gcf, 'go', true));
setappdata(previewFig, 'go', false);
while ishandle(previewFig) && ~getappdata(previewFig, 'go')
    g = rgb2gray(snapshot(cam));
    subplot(1,2,1); imshow(g); title('Live (grayscale)');
    subplot(1,2,2); imhist(g);
    title(sprintf('Min %d   Mean %.0f   Max %d', ...
                  min(g(:)), round(mean(g(:))), max(g(:))));
    drawnow;
end
if ishandle(previewFig), close(previewFig); end

boardConfirmed = ~isempty(boardSize);

H = cfg.resolution(2);
W = cfg.resolution(1);

imagePoints = {};
nCaptured   = 0;
tCalibStart = tic();
lastCapTime = -Inf;
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

    [pts, detectedSize] = detectCheckerboardPoints(frame);
    boardFound = ~isempty(pts);

    if boardFound && ~boardConfirmed
        boardSize      = detectedSize;
        boardConfirmed = true;
        fprintf('Board size auto-detected: [%d %d] interior corners. Starting captures.\n\n', ...
                boardSize(1), boardSize(2));
    end

    if boardFound && boardConfirmed
        boardFound = isequal(detectedSize, boardSize);
    end

    if boardFound && ~isempty(prevPts) && size(pts,1) == size(prevPts,1)
        drift = mean(sqrt(sum((pts - prevPts).^2, 2)));
    else
        drift = Inf;
    end
    isStable = boardFound && drift < STABILITY_THR;

    nowT       = toc(tCalibStart);
    timePassed = nowT - lastCapTime;
    if isStable && timePassed > AUTO_INTERVAL && all(isfinite(pts(:)))
        nCaptured              = nCaptured + 1;
        imagePoints{nCaptured} = pts;
        lastCapTime            = nowT;
        fprintf('Captured %d/%d\n', nCaptured, MIN_CAPTURES);
    end

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

save(saveFile, 'intrinsics');
fprintf('Saved to %s\n', saveFile);

fprintf('\nRunning holdout validation...\n');

rng(42);
n = nCaptured;
idx = randperm(n);
n_train = round(0.8 * n);
train_idx = idx(1:n_train);
holdout_idx = idx(n_train+1:end);

imagePoints_cell = squeeze(num2cell(imagePoints, [1 2]));

train_pts = cat(3, imagePoints_cell{train_idx});
holdout_pts_cell = imagePoints_cell(holdout_idx);

intrinsics_train = estimateCameraParameters( ...
    train_pts, worldPoints, ...
    'ImageSize',  [H W], ...
    'WorldUnits', 'meters');

train_err = intrinsics_train.MeanReprojectionError;
fprintf('Training subset error (%.0f%% of data): %.3fpx\n', 80, train_err);

n_holdout = numel(holdout_idx);
holdout_errors = zeros(n_holdout, 1);
valid = false(n_holdout, 1);

for i = 1:n_holdout
    pts_h = holdout_pts_cell{i};

    if isempty(pts_h) || any(~isfinite(pts_h(:)))
        continue
    end

    valid(i) = true;

    pts_undist = undistortPoints(pts_h, intrinsics_train);

    [R, t] = extrinsics(pts_undist, worldPoints, intrinsics_train);

    projected = worldToImage(intrinsics_train, R, t, ...
                             [worldPoints, zeros(size(worldPoints,1), 1)], ...
                             'ApplyDistortion', true);

    diffs = pts_h - projected;
    holdout_errors(i) = mean(sqrt(sum(diffs.^2, 2)));
end

valid_errors = holdout_errors(valid);
holdout_mean = mean(valid_errors);
ratio = holdout_mean / train_err;

fprintf('Held-out error  (%.0f%% of data): %.3fpx\n', 20, holdout_mean);
fprintf('Ratio (held-out / training):      %.2fx\n', ratio);

HELDOUT_TARGET = 0.5;
HELDOUT_LIMIT  = 1.0;
RATIO_FLAG     = 3.0;

if holdout_mean > HELDOUT_LIMIT
    fprintf(['Validation: held-out error %.3fpx exceeds the %.1fpx limit — ' ...
             'generalisation is genuinely poor. Recapture with more pose ' ...
             'variation (vary distance, tilt 30-45deg, push the board into ' ...
             'all four frame corners).\n'], holdout_mean, HELDOUT_LIMIT);
elseif holdout_mean > HELDOUT_TARGET
    if ratio > RATIO_FLAG
        fprintf(['Validation: usable (%.3fpx, under the %.1fpx limit) but ' ...
                 'ratio %.2fx is high — acceptable yet sensitive to pose. ' ...
                 'More pose diversity would firm it up.\n'], ...
                 holdout_mean, HELDOUT_LIMIT, ratio);
    else
        fprintf('Validation: good — held-out %.3fpx is within the %.1fpx limit.\n', ...
                holdout_mean, HELDOUT_LIMIT);
    end
else
    fprintf('Validation: excellent — held-out %.3fpx at or below the %.1fpx target.\n', ...
            holdout_mean, HELDOUT_TARGET);
end

figure('Name', sprintf('Holdout validation — camera %d', camIdx));
bar(valid_errors);
yline(0.5, 'g--', '0.5px target');
yline(1.0, 'r--', '1px limit');
yline(holdout_mean, 'b--', sprintf('Held-out mean %.3fpx', holdout_mean));
xlabel('Held-out capture index');
ylabel('Reprojection error (px)');
title(sprintf('Camera %d — train %.3fpx | held-out %.3fpx | ratio %.2fx', ...
    camIdx, train_err, holdout_mean, ratio));

figure('Name', sprintf('Reprojection errors — camera %d', camIdx));
bar(reprErrors);
xlabel('Capture index'); ylabel('Mean error (px)');
title(sprintf('Camera %d — mean %.3fpx', camIdx, meanErr));
yline(1.0, 'r--', '1px limit');
yline(0.5, 'g--', '0.5px target');
