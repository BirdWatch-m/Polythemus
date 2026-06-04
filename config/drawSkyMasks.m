function drawSkyMasks(cfg)
% DRAWSKYMASKS  Interactively draw a sky region mask for each camera.
%
%   drawSkyMasks(cfg)
%
%   Opens a live frame from each camera in turn and lets you draw a
%   freehand polygon defining the sky region. Only pixels inside this
%   region will be considered for bird detection. Saves all masks to
%   cfg.skyMaskFile.
%
%   Run this once after cameras are mounted in their final positions.
%   Re-run if cameras are moved or re-aimed.
%
%   CONTROLS
%     Draw  — click to place polygon vertices around the sky region
%     Close — double-click or right-click to close the polygon
%
%   INPUT
%     cfg — struct from buildConfig()
%
%   OUTPUT
%     Saves cfg.skyMaskFile containing variable 'skyMasks':
%       skyMasks — {1xN} cell of H x W logical arrays
%
%   See also: buildConfig, initSystem, gateBlobs

N = cfg.N;
H = cfg.resolution(2);
W = cfg.resolution(1);

fprintf('\nSky mask drawing — %d camera(s)\n', N);
fprintf('Draw a polygon around the sky region. Double-click to close.\n\n');

skyMasks = cell(1, N);

for i = 1:N
    fprintf('Camera %d: draw sky mask now.\n', i);

    % Open one camera at a time. We only need a single still frame per camera,
    % and two 1080p streams open at once can saturate USB bandwidth and cause
    % snapshot timeouts. Logical camera i -> physical webcamlist index.
    cam = webcam(cfg.camIndices(i));
    cam.Resolution = sprintf('%dx%d', W, H);

    % Warm-up: let the camera start streaming before the real grab, otherwise
    % the first snapshot can time out (same reason testSingleCamera warms up).
    warnState = warning('off', 'all');
    tWarm = tic();
    while toc(tWarm) < 2.0
        try
            snapshot(cam);
        catch
        end
    end
    warning(warnState);

    frame = snapshot(cam);
    cam   = [];   % release immediately — we only needed one frame

    fig = figure('Name', sprintf('Draw sky mask — camera %d', i));
    imshow(frame);
    title(sprintf('Camera %d: draw polygon around sky region. Double-click to close.', i));

    % drawpolygon lets the user click vertices; returns a logical mask.
    roi      = drawpolygon();
    skyMasks{i} = createMask(roi, frame);

    % Show result for confirmation.
    overlay        = frame;
    overlay(:,:,1) = uint8(double(frame(:,:,1)) .* 0.5 + double(skyMasks{i}) * 80);
    imshow(overlay);
    title(sprintf('Camera %d mask — close figure to continue', i));
    uiwait(fig);
end

% Save.
if ~isfolder(fileparts(cfg.skyMaskFile))
    mkdir(fileparts(cfg.skyMaskFile));
end

save(cfg.skyMaskFile, 'skyMasks');
fprintf('Sky masks saved to %s\n', cfg.skyMaskFile);

end
