function renderFrame(frames, blobs, cfg)
% RENDERFRAME Displays live frames with blob overlays.


if ~cfg.display
    return;
end

N = cfg.N;

figH = findobj('Type', 'figure', 'Name', 'BirdTracker — live');
if isempty(figH)
    figH = figure('Name', 'BirdTracker — live', 'NumberTitle', 'off', ...
                  'MenuBar', 'none', 'ToolBar', 'none');
else
    figH = figH(1);
end

hImg  = getappdata(figH, 'hImg');
hMark = getappdata(figH, 'hMark');

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

for i = 1:N
    set(hImg(i), 'CData', frames{i});

    if isempty(blobs{i})
        set(hMark(i), 'XData', NaN, 'YData', NaN);
    else
        C = vertcat(blobs{i}.centroid);
        set(hMark(i), 'XData', C(:,1), 'YData', C(:,2));
    end

    title(ancestor(hImg(i), 'axes'), sprintf('Cam %d | blobs: %d', i, numel(blobs{i})));
end

drawnow limitrate;

end
