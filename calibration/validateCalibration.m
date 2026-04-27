function results = validateCalibration(cfg)
% VALIDATECALIBRATION  Verify triangulation accuracy against a known distance.
%
%   results = validateCalibration(cfg)
%
%   Loads the saved calibration, captures a live frame from each camera,
%   lets you click the same physical point in each camera's image, then
%   triangulates it and reports the estimated distance. Compare against
%   a distance measured physically.
%
%   Run this after calibrateExtrinsics. If errors exceed 10% of the target
%   distance, repeat extrinsic calibration with more feature matches or
%   better lighting conditions.
%
%   INPUT
%     cfg     — struct from buildConfig()
%
%   OUTPUT
%     results — struct with fields:
%       .point3D          [1x3] triangulated world coordinates (metres)
%       .distFromCam1     scalar estimated distance from camera 1 (metres)
%       .reprErrors       [1xN] reprojection error per camera (pixels)
%       .clickedPixels    [Nx2] pixel coordinates you clicked per camera
%
%   PROCEDURE
%     1. Place a distinctive small object (e.g. a bright coloured marker
%        taped to a wall) at a measured distance from camera 1.
%        Measure the distance precisely with your telemeter.
%     2. Run this function.
%     3. Click the object in each camera's image when prompted.
%     4. Compare results.distFromCam1 against your measured distance.
%
%   See also: calibrateExtrinsics, calibrateIntrinsics

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
    cams{i} = webcam(i);
    cams{i}.Resolution = sprintf('%dx%d', cfg.resolution(1), cfg.resolution(2));
end

fprintf('Press any key to capture validation frames.\n');
pause;

frames = cell(1, N);
for i = 1:N
    frames{i} = snapshot(cams{i});
end
for i = 1:N, clear cams{i}; end

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

% Build cameraPose objects required by triangulateMultiview.
cameraPoses = table();
for i = 1:N
    tform = rigidtform3d(cal.R{i}, cal.t{i}');
    cameraPoses = [cameraPoses; {i, tform}];
end
cameraPoses.Properties.VariableNames = {'ViewId','AbsolutePose'};

% Build pointTrack: one track containing each camera's clicked pixel.
track = pointTrack(1:N, clickedPixels);

% Undistort clicked points using intrinsics before triangulation.
undistPts = zeros(N, 2);
for i = 1:N
    undistPts(i,:) = undistortPoints(clickedPixels(i,:), cal.intrinsics{i});
end

% triangulateMultiview expects a pointTrack and cameraPoses table.
[point3D, reprErrors] = triangulateMultiview(pointTrack(1:N, undistPts), ...
                                              cameraPoses, cal.intrinsics);

% -------------------------------------------------------------------------
% 4. REPORT
% -------------------------------------------------------------------------

distFromCam1 = norm(point3D - cal.t{1}');

fprintf('\n--- Validation results ---\n');
fprintf('Triangulated position: [%.3f  %.3f  %.3f] m\n', point3D);
fprintf('Estimated distance from camera 1: %.3f m\n', distFromCam1);
fprintf('Reprojection errors per camera: ');
fprintf('%.2fpx  ', reprErrors); fprintf('\n');
fprintf('\nMeasure this distance physically and compare.\n');
fprintf('Acceptable error: <10%% of the measured distance.\n');

results.point3D       = point3D;
results.distFromCam1  = distFromCam1;
results.reprErrors    = reprErrors;
results.clickedPixels = clickedPixels;

end
