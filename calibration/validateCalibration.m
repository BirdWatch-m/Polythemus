% VALIDATECALIBRATION  Verify triangulation accuracy against a known distance.
%
%   Script. Run with F5.
%
%   Loads the saved calibration, captures a live frame from each camera,
%   lets you click the same physical point in each camera's image, then
%   triangulates it and reports the estimated distance. Compare against
%   a distance measured physically (telemeter or tape).
%
%   Read-only check — does not modify the calibration file.
%
%   PROCEDURE
%     1. Place a distinctive small object (e.g. a bright coloured marker
%        taped to a wall) at a measured distance from camera 1.
%     2. Run this script.
%     3. Click the object in each camera's image when prompted.
%     4. Compare results.distFromCam1 against your measured distance.
%
%   THRESHOLDS
%     Reprojection errors <3px — good.
%     Reprojection errors >5px — suggests a rotation issue; recalibrate.
%     Distance error >5%      — scale not set; rerun calibrateExtrinsics
%                               with the correct knownBaseline.
%
%   OUTPUT (left in workspace as 'results')
%     .point3D          [1x3] triangulated world coordinates (metres)
%     .distFromCam1     scalar estimated distance from camera 1 (metres)
%     .reprErrors       [1xN] reprojection error per camera (pixels)
%     .clickedPixels    [Nx2] pixel coordinates clicked per camera
%
%   See also: calibrateExtrinsics, calibrateExtrinsicsCheckerboard,
%             calibrateIntrinsics

% =========================================================================
% USER INPUTS — edit these before running
% =========================================================================

clc; close all; clear;

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..')));

cfg = buildConfig();
measuredDistanceM = [14.970];   % optional: set to a telemeter/tape distance for scale diagnostics

% =========================================================================

N = cfg.N;

if ~isfile(cfg.calFile)
    error('validateCalibration:noCalFile', ...
          'Calibration file not found: %s\nRun calibrateExtrinsics first.', cfg.calFile);
end

loaded = load(cfg.calFile);
cal    = loaded.multiCamParams;
cal    = buildFundamentalMatrices(cal, N);

% -------------------------------------------------------------------------
% 1. CAPTURE FRAMES
% -------------------------------------------------------------------------

fprintf('Opening cameras...\n');
cams = cell(1, N);
for i = 1:N
    cams{i} = webcam(cfg.camIndices(i));
    cams{i}.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));
    applyCameraSettings(cams{i}, cfg);
end

fprintf('Press any key to capture validation frames.\n');
pause;

frames = cell(1, N);
for i = 1:N
    frames{i} = snapshot(cams{i});
end
cams = {};

% -------------------------------------------------------------------------
% 2. COLLECT CLICKED POINTS
% -------------------------------------------------------------------------

fprintf('\nClick the SAME physical point in each camera image.\n');
fprintf('Choose a sharp, distinct feature (corner, marker centre).\n\n');

clickedPixels = zeros(N, 2);

for i = 1:N
    figure('Name', sprintf('Click the target point — camera %d', i));
    imshow(frames{i});
    title(sprintf('Camera %d: click the target point, then press Enter', i));
    [x, y] = ginput(1);
    clickedPixels(i,:) = [x, y];
    fprintf('Camera %d: clicked (%.1f, %.1f)\n', i, x, y);
    close(gcf);
end

% -------------------------------------------------------------------------
% 3. TRIANGULATE
% -------------------------------------------------------------------------

% Build a synthetic single group and triangulate via the same DLT the live
% pipeline uses. triangulateGroups handles undistortion internally.
group.camIds = 1:N;
group.points = clickedPixels;

pt = triangulateGroups(group, cal, cfg);

point3D    = pt.position;
reprErrors = pt.reprojErrByCam;

% -------------------------------------------------------------------------
% 4. REPORT
% -------------------------------------------------------------------------

distFromCam1 = norm(point3D);

fprintf('\n--- Validation results ---\n');
fprintf('Triangulated position: [%.3f  %.3f  %.3f] m\n', point3D);
fprintf('Estimated distance from camera 1: %.3f m\n', distFromCam1);
fprintf('Mean reprojection error: %.2fpx\n', pt.reprojErr);
fprintf('Reprojection errors per camera: ');
fprintf('%.2fpx  ', reprErrors); fprintf('\n');

if ~isempty(measuredDistanceM)
    distanceErr = distFromCam1 - measuredDistanceM;
    scaleFactor = measuredDistanceM / distFromCam1;
    fprintf('Physical distance: %.3f m\n', measuredDistanceM);
    fprintf('Distance error: %+.3f m (%+.1f%%)\n', ...
            distanceErr, 100 * distanceErr / measuredDistanceM);
    fprintf('Scale factor that would match this point: %.6f\n', scaleFactor);
end

clickedUndist = zeros(N, 2);
for i = 1:N
    clickedUndist(i,:) = undistortPoints(clickedPixels(i,:), cal.intrinsics{i});
end

fprintf('\n--- Click consistency diagnostics ---\n');
for i = 1:N
    delta = pt.reprojectedPoints(i,:) - clickedUndist(i,:);
    fprintf(['Cam %d: raw click [%.2f %.2f] | undist [%.2f %.2f] | ' ...
             'reprojected [%.2f %.2f] | correction [%+.2f %+.2f] px | err %.2fpx\n'], ...
            i, clickedPixels(i,1), clickedPixels(i,2), ...
            clickedUndist(i,1), clickedUndist(i,2), ...
            pt.reprojectedPoints(i,1), pt.reprojectedPoints(i,2), ...
            delta(1), delta(2), reprErrors(i));
end

if N == 2
    epiRaw = symmetricEpipolarDistance(clickedPixels(1,:), clickedPixels(2,:), cal.F{1,2});
    epiUndist = symmetricEpipolarDistance(clickedUndist(1,:), clickedUndist(2,:), cal.F{1,2});
    fprintf('Symmetric epipolar distance, raw clicks: %.2fpx\n', epiRaw);
    fprintf('Symmetric epipolar distance, undistorted clicks: %.2fpx\n', epiUndist);
end

fprintf('\nCompare distance against a physical measurement.\n');
fprintf('Reprojection errors <3px are good; >5px suggests a rotation issue.\n');
fprintf('If the correction vectors are comparable to click uncertainty, redo with a sharper/zoomed target.\n');
fprintf('If they stay tens of pixels on a sharp target, treat the extrinsic rotation as bad.\n');

results.point3D       = point3D;
results.distFromCam1  = distFromCam1;
results.reprErrors    = reprErrors;
results.clickedPixels = clickedPixels;
results.clickedUndistorted = clickedUndist;
results.reprojectedPoints  = pt.reprojectedPoints;
results.reprojectionDeltas = pt.reprojectedPoints - clickedUndist;
if N == 2
    results.epipolarDistanceRaw = epiRaw;
    results.epipolarDistanceUndistorted = epiUndist;
end


function d = symmetricEpipolarDistance(x1, x2, F)
p1   = [x1(1); x1(2); 1];
p2   = [x2(1); x2(2); 1];
Fp1  = F   * p1;
Ftp2 = F.' * p2;
num  = abs(p2.' * Fp1);
dj   = num / hypot(Fp1(1),  Fp1(2));
di   = num / hypot(Ftp2(1), Ftp2(2));
d    = 0.5 * (di + dj);
end
