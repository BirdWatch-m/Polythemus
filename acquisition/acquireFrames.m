function [frames, timestamps] = acquireFrames(cams, tStart, cfg)
% ACQUIREFRAMES Captures one frame per camera and timestamps each capture.


N = cfg.N;

frames     = cell(1, N);
timestamps = zeros(1, N);

for i = 1:N
    frames{i}     = snapshot(cams{i});
    timestamps(i) = toc(tStart);
end

end
