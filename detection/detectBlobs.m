function [blobs, state, counts] = detectBlobs(grayFrames, state, cfg)
% DETECTBLOBS  Run per-camera preprocessing for all N cameras.
%
%   [blobs, state, counts] = detectBlobs(grayFrames, state, cfg)
%
%   Loops over cameras, calling preprocessFrame for each. Updates
%   bgMedian in state (fgDetectors update internally via handle objects).
%   Takes grayscale frames (from updateRingBuf) so the rgb2gray is not repeated.
%
%   INPUTS
%     grayFrames — {1xN} cell of H x W uint8 grayscale frames (from updateRingBuf)
%     state      — system state struct from initSystem
%     cfg        — struct from buildConfig()
%
%   OUTPUTS
%     blobs  — {1xN} cell of struct arrays (one per camera, see gateBlobs)
%     state  — updated state (bgMedian and bgFramesSinceUpdate fields updated)
%     counts — struct with detection gate totals summed across all cameras:
%              rawRegions, rejSmall, rejLarge, rejAspect, passed
%
%   See also: preprocessFrame, updateRingBuf, acquireFrames, associateViews

N      = cfg.N;
blobs  = cell(1, N);
counts = struct('rawRegions', 0, 'rejSmall', 0, 'rejLarge', 0, 'rejAspect', 0, 'passed', 0);

for i = 1:N
    [blobs{i}, state.bgMedian{i}, state.bgFramesSinceUpdate(i), dc] = preprocessFrame( ...
        grayFrames{i},      ...
        state.bgMedian{i},  ...
        state.ringBuf{i},   ...
        state.bgFramesSinceUpdate(i), ...
        state.fgDetectors{i}, ...
        state.skyMask{i},   ...
        cfg);
    counts.rawRegions = counts.rawRegions + dc.rawRegions;
    counts.rejSmall   = counts.rejSmall   + dc.rejSmall;
    counts.rejLarge   = counts.rejLarge   + dc.rejLarge;
    counts.rejAspect  = counts.rejAspect  + dc.rejAspect;
    counts.passed     = counts.passed     + dc.passed;
end

end
