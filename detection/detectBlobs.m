function [blobs, state, counts] = detectBlobs(grayFrames, state, cfg)
% DETECTBLOBS Runs blob detection on all cameras.


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
