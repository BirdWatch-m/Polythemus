function drawSkyMasks(cfg)
% DRAWSKYMASKS Draws and saves per-camera sky masks.


N = cfg.N;
H = cfg.resolution(2);
W = cfg.resolution(1);

fprintf('\nSky mask drawing — %d camera(s)\n', N);
fprintf('Draw a polygon around the sky region. Double-click to close.\n\n');

skyMasks = cell(1, N);

for i = 1:N
    fprintf('Camera %d: draw sky mask now.\n', i);

    cam = webcam(cfg.camIndices(i));
    cam.Resolution = sprintf('%dx%d', W, H);
    applyCameraSettings(cam, cfg);

    frame = snapshot(cam);
    cam   = [];

    fig = figure('Name', sprintf('Draw sky mask — camera %d', i));
    imshow(frame);
    title(sprintf('Camera %d: draw polygon around sky region. Double-click to close.', i));

    roi      = drawpolygon();
    skyMasks{i} = createMask(roi, frame);

    overlay        = frame;
    overlay(:,:,1) = uint8(double(frame(:,:,1)) .* 0.5 + double(skyMasks{i}) * 80);
    imshow(overlay);
    title(sprintf('Camera %d mask — close figure to continue', i));
    uiwait(fig);
end

if ~isfolder(fileparts(cfg.skyMaskFile))
    mkdir(fileparts(cfg.skyMaskFile));
end

save(cfg.skyMaskFile, 'skyMasks');
fprintf('Sky masks saved to %s\n', cfg.skyMaskFile);

end
