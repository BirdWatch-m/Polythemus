function [blobs, state] = detectBlobs(grayFrames, state, cfg)
% DETECTBLOBS  Run per-camera preprocessing for all N cameras.
%
%   [blobs, state] = detectBlobs(grayFrames, state, cfg)
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
%     blobs — {1xN} cell of struct arrays (one per camera, see gateBlobs)
%     state — updated state (bgMedian and bgFramesSinceUpdate fields updated)
%
%   See also: preprocessFrame, updateRingBuf, acquireFrames, associateViews

N     = cfg.N;
blobs = cell(1, N);

for i = 1:N
    [blobs{i}, state.bgMedian{i}, state.bgFramesSinceUpdate(i)] = preprocessFrame( ...
        grayFrames{i},      ...
        state.bgMedian{i},  ...
        state.ringBuf{i},   ...
        state.bgFramesSinceUpdate(i), ...
        state.fgDetectors{i}, ...
        state.skyMask{i},   ...
        cfg);
end

end
