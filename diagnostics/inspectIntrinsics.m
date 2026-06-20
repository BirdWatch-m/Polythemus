function inspectIntrinsics(files)
% INSPECTINTRINSICS  Report intrinsic-calibration quality and distortion footprint.
%
%   inspectIntrinsics()           % inspects the two 720p operating intrinsics
%   inspectIntrinsics({...paths}) % inspects the given .mat files
%
%   For each file: model parameters, stored calibration reprojection error,
%   the IntrinsicMatrix/K transpose convention the pipeline relies on, and how
%   far undistortPoints displaces pixels across the frame. A large peripheral
%   displacement means the distortion model carries most of the geometry near
%   the edges — so any under-constrained edge (board never pushed into the
%   corners) leaves a large, FIXED residual there: exactly the kind of
%   repeatable multi-pixel reprojection error that survives every recalibration.
%
%   Read-only. No hardware required.
%
%   See also: calibrateIntrinsics, validateCalibration, triangulateGroups

if nargin < 1 || isempty(files)
    files = {'calibration/intrinsics_MY1_720.mat', ...
             'calibration/intrinsics_LG1_720.mat'};
end
for i = 1:numel(files)
    inspectOne(files{i});
end
end


function inspectOne(f)
fprintf('\n===== %s =====\n', f);
if ~isfile(f), fprintf('  (missing)\n'); return; end
S  = load(f); fn = fieldnames(S); obj = S.(fn{1});
fprintf('  class: %s\n', class(obj));

if isa(obj, 'cameraParameters')
    intr = obj.Intrinsics; cp = obj;
else
    intr = obj; cp = [];
end

IS = intr.ImageSize;                 % [rows cols] = [H W]
H = IS(1); W = IS(2);
fprintf('  ImageSize [rows cols] : [%d %d]   (720p expects [720 1280])\n', H, W);
fprintf('  FocalLength (px)      : [%.2f %.2f]\n', intr.FocalLength);
fprintf('  PrincipalPoint (px)   : [%.2f %.2f]   (frame centre [%.1f %.1f])\n', ...
        intr.PrincipalPoint, W/2, H/2);
fprintf('  PrincipalPoint offset : [%+.1f %+.1f] px from centre\n', ...
        intr.PrincipalPoint(1)-W/2, intr.PrincipalPoint(2)-H/2);
fprintf('  RadialDistortion      : %s\n', mat2str(intr.RadialDistortion, 4));
fprintf('  TangentialDistortion  : %s\n', mat2str(intr.TangentialDistortion, 4));
if ~isempty(cp)
    fprintf('  MeanReprojectionError : %.3f px   (NumPatterns %d, units %s)\n', ...
            cp.MeanReprojectionError, cp.NumPatterns, cp.WorldUnits);
else
    fprintf('  MeanReprojectionError : (not stored — object is cameraIntrinsics)\n');
end

% Confirm the transpose convention the pipeline assumes: K = IntrinsicMatrix.'
Kstd = intr.K; Kim = intr.IntrinsicMatrix;
fprintf('  IntrinsicMatrix.'' == K (standard)? %d\n', ...
        max(abs(Kim.' - Kstd), [], 'all') < 1e-6);

% Distortion footprint: displacement undistortPoints applies across the frame.
ctr = [W/2 H/2];
edgeMid = [W/2 1; W/2 H; 1 H/2; W H/2];
corners = [1 1; W 1; 1 H; W H];
dCtr  = disp_(ctr,     intr);
dEdge = disp_(edgeMid, intr);
dCorn = disp_(corners, intr);
fprintf('  undistort displacement: centre %.1f px | edge-mid max %.1f px | CORNER max %.1f px\n', ...
        dCtr, max(dEdge), max(dCorn));
fprintf('  corner displacements  : %s px\n', mat2str(round(dCorn(:).'), 4));
if max(dCorn) > 25
    fprintf(['  >> Peripheral distortion is LARGE. If the calibration board did not\n' ...
             '     reach the corners, residual model error there can be many px and\n' ...
             '     will repeat on every run. Check coverage / recapture into corners.\n']);
end
end


function d = disp_(pts, intr)
u = undistortPoints(pts, intr);
d = sqrt(sum((u - pts).^2, 2));
end
