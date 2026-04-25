function [ringBuf, ringIdx] = updateRingBuf(ringBuf, ringIdx, frames, cfg)
% UPDATERINGBUF  Insert current frames into the per-camera ring buffers.
%
%   [ringBuf, ringIdx] = updateRingBuf(ringBuf, ringIdx, frames, cfg)
%
%   The ring buffer is a circular store of the last cfg.ringBufLen frames.
%   Once full, new frames overwrite the oldest. The write pointer ringIdx
%   advances by 1 each call and wraps back to 1 after ringBufLen.
%
%   INPUTS
%     ringBuf — {1xN} cell of H x W x ringBufLen uint8 (current buffer)
%     ringIdx — [1xN] current write position per camera
%     frames  — {1xN} cell of H x W x 3 uint8 (current colour frames)
%     cfg     — struct from buildConfig()
%
%   OUTPUTS
%     ringBuf — updated buffer
%     ringIdx — updated write positions
%
%   See also: acquireFrames, applyBackground

N = cfg.N;

for i = 1:N
    % Convert colour frame to grayscale before storing — detection uses gray only.
    ringBuf{i}(:,:,ringIdx(i)) = rgb2gray(frames{i});

    % Advance write pointer, wrapping at buffer length.
    ringIdx(i) = mod(ringIdx(i), cfg.ringBufLen) + 1;
end

end
