function renderFrame(frames, blobs, cfg)
% RENDERFRAME  Display live camera feeds with blob detection overlay.
%
%   renderFrame(frames, blobs, cfg)
%
%   Shows all N camera feeds side by side. Detected blobs are marked
%   with green circles and their pixel area printed alongside.
%   No-op if cfg.display is false.
%
%   This function has no return values and no effect on system state.
%
%   INPUTS
%     frames — {1xN} cell of H x W x 3 uint8 colour frames
%     blobs  — {1xN} cell of blob struct arrays from detectBlobs
%     cfg    — struct from buildConfig()
%
%   See also: detectBlobs, logFrame

if ~cfg.display
    return;
end

N = cfg.N;

% Create or reuse figure.
fig = findobj('Type', 'figure', 'Name', 'BirdTracker — live');
if isempty(fig)
    fig = figure('Name', 'BirdTracker — live', 'NumberTitle', 'off', ...
             'MenuBar', 'none', 'ToolBar', 'none');
end

for i = 1:N
    subplot(1, N, i);

    display = frames{i};

    % Draw a green circle at each detected blob centroid.
    for k = 1:numel(blobs{i})
        c       = blobs{i}(k).centroid;
        display = insertShape(display, 'Circle', [c(1) c(2) 6], ...
                              'Color', 'green', 'LineWidth', 2);
        display = insertText(display, [c(1)+8 c(2)], ...
                             sprintf('%.0fpx²', blobs{i}(k).area), ...
                             'FontSize', 10, 'BoxOpacity', 0);
    end

    image(display);
    axis off;
    title(sprintf('Cam %d | blobs: %d', i, numel(blobs{i})));
end

drawnow limitrate;

end
