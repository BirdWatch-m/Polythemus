function [ringBuf, ringIdx, grayFrames] = updateRingBuf(ringBuf, ringIdx, frames, cfg)
% UPDATERINGBUF Updates ring buffers and returns grayscale frames.


N = cfg.N;

grayFrames = cell(1, N);

for i = 1:N
    if size(frames{i}, 3) == 3
        grayFrames{i} = rgb2gray(frames{i});
    else
        grayFrames{i} = frames{i};
    end
    ringBuf{i}(:,:,ringIdx(i)) = grayFrames{i};

    ringIdx(i) = mod(ringIdx(i), cfg.ringBufLen) + 1;
end

end
