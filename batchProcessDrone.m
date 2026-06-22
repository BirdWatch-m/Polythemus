% BATCHPROCESSDRONE  Run tuned detection on all drone recordings.
%
%   Processes each recording in full with the validated parameter overrides
%   (useMorphology=false, minBlobArea=4). Saves results_tuned.mat and
%   cam{1,2}_replay_tuned.avi alongside the originals without overwriting them.

clc; close all; clear;
addpath(genpath(fileparts(mfilename('fullpath'))));

recordings = {
    'output/recordings/20260618_173744',
    'output/recordings/20260618_174234',
    'output/recordings/20260619_201550',
};

summary = struct();

for ri = 1:numel(recordings)
    recDir = recordings{ri};
    [~, tag] = fileparts(recDir);
    fprintf('\n========================================\n');
    fprintf('RECORDING: %s\n', tag);
    fprintf('========================================\n');

    % ---- Load session ----
    loaded  = load(fullfile(recDir, 'session.mat'));
    session = loaded.session;
    cfg     = session.cfg;

    % ---- Parameter overrides ----
    cfg.useMorphology = false;
    cfg.minBlobArea   = 4;

    N       = cfg.N;
    H       = cfg.resolution(2);
    W       = cfg.resolution(1);
    nFrames = session.nFrames;

    fprintf('Frames: %d  Duration: %.1fs\n', nFrames, ...
            session.log.timestamps(1,end) - session.log.timestamps(1,1));
    fprintf('Overrides: useMorphology=false  minBlobArea=4\n');

    % ---- Init state ----
    state.ringBuf             = cell(1,N);
    state.ringIdx             = ones(1,N);
    state.bgMedian            = cell(1,N);
    state.bgFramesSinceUpdate = zeros(1,N);
    state.fgDetectors         = cell(1,N);
    state.skyMask             = cell(1,N);
    for i = 1:N
        state.ringBuf{i}     = zeros(H,W,cfg.ringBufLen,'uint8');
        state.bgMedian{i}    = zeros(H,W,'int16');
        state.fgDetectors{i} = vision.ForegroundDetector('NumGaussians',3, ...
            'NumTrainingFrames',60,'LearningRate',0.005);
        state.skyMask{i}     = true(H,W);
    end

    % ---- Calibration (session-local only — no fallback) ----
    sessionCalFile = fullfile(recDir,'multiCamParams.mat');
    if isfield(session,'calibration') && ~isempty(session.calibration)
        multiCamParams = session.calibration;
        fprintf('Calibration: embedded in session.mat\n');
    elseif isfile(sessionCalFile)
        c = load(sessionCalFile); multiCamParams = c.multiCamParams;
        fprintf('Calibration: session-local %s\n', sessionCalFile);
    else
        error('batchProcessDrone:noCalib', 'No local calibration in %s — skipping', recDir);
    end
    calibration = buildFundamentalMatrices(multiCamParams, N);
    tracks  = struct('id',{},'state',{},'kf',{},'age',{},'noDetAge',{},'lastPos',{});
    nextId  = 1;
    ts      = mean(session.log.timestamps,1);
    dts     = [1/cfg.fps, diff(ts)];

    % ---- Results arrays ----
    results.nBlobs         = zeros(N, nFrames);
    results.blobCentroids  = cell(1, nFrames);
    results.nGroups        = zeros(1, nFrames);
    results.nPoints        = zeros(1, nFrames);
    results.meanReprErr    = nan(1, nFrames);
    results.nConfirmed     = zeros(1, nFrames);
    results.trackIds       = cell(1, nFrames);
    results.trackPositions = cell(1, nFrames);
    results.frameIndex     = 1:nFrames;

    % ---- Process ----
    fprintf('Processing...\n');
    for k = 1:nFrames
        frames = cell(1,N);
        for i = 1:N
            frames{i} = imread(fullfile(recDir,sprintf('cam%d',i),sprintf('frame_%06d.tif',k)));
        end
        [state.ringBuf,state.ringIdx,grayFrames] = updateRingBuf(state.ringBuf,state.ringIdx,frames,cfg);
        [blobs,state] = detectBlobs(grayFrames,state,cfg);
        results.nBlobs(:,k)    = cellfun(@numel,blobs).';
        results.blobCentroids{k} = cellfun(@(b) vertcat(b.centroid),blobs,'UniformOutput',false);

        groups = associateViews(blobs,calibration,cfg);
        points = triangulateGroups(groups,calibration,cfg);
        valid  = points([points.valid]);
        if isempty(valid), meas = zeros(0,3); else, meas = vertcat(valid.position); end
        [tracks,nextId] = updateTracks(tracks,nextId,meas,dts(k),cfg);

        results.nGroups(k)     = numel(groups);
        results.nPoints(k)     = numel(valid);
        results.meanReprErr(k) = mean([valid.reprojErr]);
        results.nConfirmed(k)  = sum(strcmp({tracks.state},'confirmed'));
        confirmed = tracks(strcmp({tracks.state},'confirmed'));
        if isempty(confirmed)
            results.trackIds{k}       = zeros(0,1);
            results.trackPositions{k} = zeros(0,3);
        else
            results.trackIds{k}       = vertcat(confirmed.id);
            results.trackPositions{k} = vertcat(confirmed.lastPos);
        end

        if mod(k,100)==0
            fprintf('  %4d/%d | blobs %s | pts %d | confirmed %d\n', ...
                k, nFrames, mat2str(results.nBlobs(:,k).'), results.nPoints(k), results.nConfirmed(k));
        end
    end
    results.finalTracks = tracks;

    % ---- Save results ----
    outFile = fullfile(recDir,'results_tuned.mat');
    save(outFile,'results');

    % ---- Summary stats ----
    oldFile = fullfile(recDir,'results.mat');
    hasOld  = isfile(oldFile);
    fprintf('\n--- %s ---\n', tag);
    if hasOld
        old = load(oldFile); old = old.results;
        fprintf('                      BEFORE       AFTER\n');
        fprintf('cam1 blob frames:   %6d       %6d\n', sum(old.nBlobs(1,:)>=1), sum(results.nBlobs(1,:)>=1));
        fprintf('cam2 blob frames:   %6d       %6d\n', sum(old.nBlobs(2,:)>=1), sum(results.nBlobs(2,:)>=1));
        if isfield(old,'nConfirmed')
            oldErr = old.meanReprErr(~isnan(old.meanReprErr));
            newErr = results.meanReprErr(~isnan(results.meanReprErr));
            fprintf('confirmed frames:   %6d       %6d\n', sum(old.nConfirmed>=1), sum(results.nConfirmed>=1));
            fprintf('max confirmed/frm:  %6d       %6d\n', max(old.nConfirmed), max(results.nConfirmed));
            if ~isempty(oldErr), fprintf('mean reproj err:    %.2fpx      ', mean(oldErr)); else, fprintf('mean reproj err:       N/A      '); end
            if ~isempty(newErr), fprintf('%.2fpx\n', mean(newErr)); else, fprintf('N/A\n'); end
        end
    else
        fprintf('(no original results.mat to compare)\n');
        fprintf('cam1 blob frames: %d / %d\n', sum(results.nBlobs(1,:)>=1), nFrames);
        fprintf('cam2 blob frames: %d / %d\n', sum(results.nBlobs(2,:)>=1), nFrames);
        fprintf('confirmed frames: %d\n', sum(results.nConfirmed>=1));
        fprintf('max confirmed/frame: %d\n', max(results.nConfirmed));
        validErr = results.meanReprErr(~isnan(results.meanReprErr));
        if ~isempty(validErr), fprintf('mean reproj err: %.2fpx\n', mean(validErr)); end
    end

    summary(ri).tag          = tag;
    summary(ri).nFrames      = nFrames;
    summary(ri).cam1Tuned    = sum(results.nBlobs(1,:)>=1);
    summary(ri).cam2Tuned    = sum(results.nBlobs(2,:)>=1);
    summary(ri).confirmedTuned = sum(results.nConfirmed>=1);
    summary(ri).maxConfirmed = max(results.nConfirmed);
    e = results.meanReprErr(~isnan(results.meanReprErr));
    summary(ri).reprErr      = mean(e);

    % ---- Render video ----
    fprintf('\nRendering videos...\n');
    for i = 1:N
        videoOut = fullfile(recDir, sprintf('cam%d_replay_tuned.avi',i));
        vw = VideoWriter(videoOut); vw.FrameRate = 30; open(vw);
        for k = 1:nFrames
            frame = imread(fullfile(recDir,sprintf('cam%d',i),sprintf('frame_%06d.tif',k)));
            t     = session.log.timestamps(i,k);
            syncMs = session.log.syncMs(k);
            infoStr = sprintf('cam%d | f%d | t=%.2fs | blobs:%d | confirmed:%d | reprErr:%.1fpx [TUNED minA=4 morphOff]', ...
                              i, k, t, results.nBlobs(i,k), results.nConfirmed(k), results.meanReprErr(k));
            ann = repmat(frame,[1 1 3]);
            ann = insertText(ann,[10 10],infoStr,'FontSize',12,'BoxColor','black','TextColor','yellow','BoxOpacity',0.6);
            cents = results.blobCentroids{k}{i};
            if ~isempty(cents)
                ann = insertShape(ann,'Circle',[cents, repmat(8,size(cents,1),1)],'Color','green','LineWidth',2);
            end
            writeVideo(vw,ann);
        end
        close(vw);
        fprintf('  Saved: %s\n', videoOut);
    end
end

fprintf('\n\n========== FINAL SUMMARY ==========\n');
fprintf('%-22s  %5s  %5s  %5s  %8s  %8s  %7s\n', ...
        'Recording','Frms','c1blb','c2blb','conf_frms','max_conf','reprErr');
for ri = 1:numel(summary)
    s = summary(ri);
    fprintf('%-22s  %5d  %5d  %5d  %8d  %8d  %.2fpx\n', ...
            s.tag, s.nFrames, s.cam1Tuned, s.cam2Tuned, s.confirmedTuned, s.maxConfirmed, s.reprErr);
end
