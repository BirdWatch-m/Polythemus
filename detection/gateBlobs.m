function blobs = gateBlobs(mask, skyMask, cfg)
% GATEBLOBS  Extract blobs from foreground mask and apply detection filters.
%
%   blobs = gateBlobs(mask, skyMask, cfg)
%
%   Applies the sky mask, runs morphological cleanup, extracts connected
%   regions, then filters by area and aspect ratio. Returns only blobs
%   that pass all filters.
%
%   INPUTS
%     mask    — H x W logical foreground mask from applyBackground
%     skyMask — H x W logical sky region mask from drawSkyMasks
%     cfg     — struct from buildConfig()
%
%   OUTPUT
%     blobs — struct array, each element has fields:
%               .centroid  [u v] pixel coordinates (double)
%               .area      scalar pixel area (double)
%               .bbox      [x y width height] bounding box (double)
%
%   See also: applyBackground, preprocessFrame, associateViews

% Restrict detection to sky region only.
mask = mask & skyMask;

% Morphological cleanup: remove isolated noise pixels, then fill small gaps.
se   = strel('disk', 2);
mask = imopen(mask, se);    % erosion then dilation — removes speckle
mask = imclose(mask, se);   % dilation then erosion — fills holes within blobs

% Extract connected regions and their properties.
stats = regionprops(mask, 'Centroid', 'Area', 'BoundingBox', 'MajorAxisLength', 'MinorAxisLength');

% Filter by area.
validArea = [stats.Area] >= cfg.minBlobArea & [stats.Area] <= cfg.maxBlobArea;
stats     = stats(validArea);

% Filter by aspect ratio — avoids compact noise and very diffuse cloud patches.
if ~isempty(stats)
    minorAxes   = [stats.MinorAxisLength];
    majorAxes   = [stats.MajorAxisLength];
    safeMinor   = max(minorAxes, 0.1);   % avoid division by zero
    aspects     = majorAxes ./ safeMinor;
    validAspect = aspects <= cfg.maxAspect;
    stats       = stats(validAspect);
end

% Build output struct array.
nBlobs = numel(stats);
if nBlobs == 0
    blobs = struct('centroid', {}, 'area', {}, 'bbox', {});
    return;
end

blobs(nBlobs) = struct('centroid', [], 'area', [], 'bbox', []);
for k = 1:nBlobs
    blobs(k).centroid = stats(k).Centroid;
    blobs(k).area     = stats(k).Area;
    blobs(k).bbox     = stats(k).BoundingBox;
end

end
