function [frames, timestamps] = acquireFrames(cams, tStart, cfg)
% ACQUIREFRAMES  Grab one frame from each camera as close in time as possible.
%
%   [frames, timestamps] = acquireFrames(cams, tStart, cfg)
%
%   INPUTS
%     cams      — {1xN} cell of webcam objects from initSystem
%     tStart    — tic reference from initSystem (state.tStart)
%     cfg       — struct from buildConfig()
%
%   OUTPUTS
%     frames     — {1xN} cell of H x W x 3 uint8 images
%     timestamps — [1xN] double, seconds since tStart per camera
%
%   See also: syncCheck, updateRingBuf, initSystem

N = cfg.N;

frames     = cell(1, N);
timestamps = zeros(1, N);

for i = 1:N
    frames{i}     = snapshot(cams{i});
    timestamps(i) = toc(tStart);
end

end
