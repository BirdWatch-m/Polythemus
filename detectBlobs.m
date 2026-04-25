function [blobs, state] = detectBlobs(frames, state, cfg)
% DETECTBLOBS  Run per-camera preprocessing for all N cameras.
%
%   [blobs, state] = detectBlobs(frames, state, cfg)
%
%   Loops over cameras, calling preprocessFrame for each. Updates
%   bgMedian in state (fgDetectors update internally via handle objects).
%
%   INPUTS
%     frames — {1xN} cell of H x W x 3 uint8 colour frames
%     state  — system state struct from initSystem
%     cfg    — struct from buildConfig()
%
%   OUTPUTS
%     blobs — {1xN} cell of struct arrays (one per camera, see gateBlobs)
%     state — updated state (bgMedian fields updated)
%
%   See also: preprocessFrame, acquireFrames, associateViews

N     = cfg.N;
blobs = cell(1, N);

for i = 1:N
    [blobs{i}, state.bgMedian{i}] = preprocessFrame( ...
        frames{i},          ...
        state.bgMedian{i},  ...
        state.ringBuf{i},   ...
        state.fgDetectors{i}, ...
        state.skyMask{i},   ...
        cfg);
end

end
