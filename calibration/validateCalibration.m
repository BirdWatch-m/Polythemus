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

% =========================================================================

N = cfg.N;

if ~isfile(cfg.calFile)
    error('validateCalibration:noCalFile', ...
          'Calibration file not found: %s\nRun calibrateExtrinsics first.', cfg.calFile);
end

loaded = load(cfg.calFile);
cal    = loaded.multiCamParams;

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
reprErrors = pt.reprojErr;

% -------------------------------------------------------------------------
% 4. REPORT
% -------------------------------------------------------------------------

distFromCam1 = norm(point3D);

fprintf('\n--- Validation results ---\n');
fprintf('Triangulated position: [%.3f  %.3f  %.3f] m\n', point3D);
fprintf('Estimated distance from camera 1: %.3f m\n', distFromCam1);
fprintf('Reprojection errors per camera: ');
fprintf('%.2fpx  ', reprErrors); fprintf('\n');
fprintf('\nCompare distance against a physical measurement.\n');
fprintf('Reprojection errors <3px are good; >5px suggests a rotation issue.\n');

results.point3D       = point3D;
results.distFromCam1  = distFromCam1;
results.reprErrors    = reprErrors;
results.clickedPixels = clickedPixels;
