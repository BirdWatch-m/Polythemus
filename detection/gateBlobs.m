function [blobs, counts] = gateBlobs(mask, skyMask, cfg)
% GATEBLOBS  Extract blobs from foreground mask and apply detection filters.
%
%   [blobs, counts] = gateBlobs(mask, skyMask, cfg)
%
%   Applies the sky mask, runs morphological cleanup, extracts connected
%   regions, then filters by area and aspect ratio. Returns only blobs
%   that pass all filters, plus a counts struct for gate reporting.
%
%   INPUTS
%     mask    — H x W logical foreground mask from applyBackground
%     skyMask — H x W logical sky region mask from drawSkyMasks
%     cfg     — struct from buildConfig()
%
%   OUTPUTS
%     blobs  — struct array, each element has fields:
%                .centroid  [u v] pixel coordinates (double)
%                .area      scalar pixel area (double)
%                .bbox      [x y width height] bounding box (double)
%     counts — struct: rawRegions, rejSmall, rejLarge, rejAspect, passed
%
%   See also: applyBackground, preprocessFrame, associateViews

% Restrict detection to sky region only.
if ~isequal(size(mask), size(skyMask))
    error('gateBlobs:maskSizeMismatch', ...
          ['Sky mask is %dx%d but the frame is %dx%d.\n' ...
           'Re-run drawSkyMasks at the current resolution.'], ...
          size(skyMask,1), size(skyMask,2), size(mask,1), size(mask,2));
end
mask = mask & skyMask;

if cfg.useMorphology
    mask = imopen(mask, cfg.morphStrel);
    mask = imclose(mask, cfg.morphStrel);
end

% Extract connected regions.
stats = regionprops(mask, 'Centroid', 'Area', 'BoundingBox', 'MajorAxisLength', 'MinorAxisLength');

% Count raw regions before any gate.
rawRegions = numel(stats);

% Filter by area — count rejections before removing.
if rawRegions > 0
    areas    = [stats.Area];
    rejSmall = sum(areas < cfg.minBlobArea);
    rejLarge = sum(areas > cfg.maxBlobArea);
    stats    = stats(areas >= cfg.minBlobArea & areas <= cfg.maxBlobArea);
else
    rejSmall = 0;
    rejLarge = 0;
end

% Filter by aspect ratio — count rejections before removing.
rejAspect = 0;
if ~isempty(stats)
    safeMinor = max([stats.MinorAxisLength], 0.1);
    aspects   = [stats.MajorAxisLength] ./ safeMinor;
    validAsp  = aspects <= cfg.maxAspect;
    rejAspect = sum(~validAsp);
    stats     = stats(validAsp);
end

nBlobs = numel(stats);
counts = struct('rawRegions', rawRegions, 'rejSmall', rejSmall, ...
                'rejLarge',   rejLarge,   'rejAspect', rejAspect, 'passed', nBlobs);

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
