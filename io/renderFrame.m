function renderFrame(frames, blobs, cfg)
% RENDERFRAME  Display live camera feeds with blob detection overlay.
%
%   renderFrame(frames, blobs, cfg)
%
%   Shows all N camera feeds side by side; detected blobs are marked with
%   green circles. No-op if cfg.display is false.
%
%   For speed, the image and marker graphics are created ONCE and then updated
%   in place (CData / XData / YData) on later calls. Rebuilding the subplots and
%   rasterising overlays into the frame every iteration (the old approach) cost
%   ~100ms/frame even with no blobs; updating in place is ~an order cheaper.
%
%   INPUTS
%     frames — {1xN} cell of H x W x 3 uint8 colour frames
%     blobs  — {1xN} cell of blob struct arrays from detectBlobs
%     cfg    — struct from buildConfig()
%
%   This function has no return values and no effect on system state.
%
%   See also: detectBlobs, logFrame

if ~cfg.display
    return;
end

N = cfg.N;

% Find (or create) the live figure. main.m / testSingleCamera create it with the
% Q-key handler; reuse it so that handler stays attached.
figH = findobj('Type', 'figure', 'Name', 'BirdTracker — live');
if isempty(figH)
    figH = figure('Name', 'BirdTracker — live', 'NumberTitle', 'off', ...
                  'MenuBar', 'none', 'ToolBar', 'none');
else
    figH = figH(1);
end

hImg  = getappdata(figH, 'hImg');
hMark = getappdata(figH, 'hMark');

% First call (or stale handles): build axes, image, and marker objects once.
if isempty(hImg) || numel(hImg) ~= N || ~all(isgraphics(hImg))
    hImg  = gobjects(1, N);
    hMark = gobjects(1, N);
    for i = 1:N
        ax = subplot(1, N, i, 'Parent', figH);
        hImg(i)  = imshow(frames{i}, 'Parent', ax);
        hold(ax, 'on');
        hMark(i) = plot(ax, NaN, NaN, 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
        title(ax, sprintf('Cam %d', i));
    end
    setappdata(figH, 'hImg',  hImg);
    setappdata(figH, 'hMark', hMark);
end

% Update image data and blob markers in place.
for i = 1:N
    set(hImg(i), 'CData', frames{i});

    if isempty(blobs{i})
        set(hMark(i), 'XData', NaN, 'YData', NaN);
    else
        C = vertcat(blobs{i}.centroid);            % nBlobs x 2
        set(hMark(i), 'XData', C(:,1), 'YData', C(:,2));
    end

    title(ancestor(hImg(i), 'axes'), sprintf('Cam %d | blobs: %d', i, numel(blobs{i})));
end

drawnow limitrate;

end
