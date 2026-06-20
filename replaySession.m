% REPLAYSESSION  Write annotated video(s) from a recorded session.
%
%   Script. Set recordingDir below, then run with F5.
%   Reads raw frames saved by recordSession and blob counts from results.mat
%   (written by processRecording), and writes one annotated video per camera
%   to the recording directory.
%
%   Run processRecording on the session first to get blob counts in the
%   annotation. Without results.mat the video is annotated with timestamps
%   and sync timing only.
%
%   See also: recordSession, processRecording

clc; close all; clear;

addpath(genpath(fileparts(mfilename('fullpath'))));

% =========================================================================
% USER INPUTS
% =========================================================================
recordingDir = 'output/recordings/20260619_201550';
playbackFps  = 30;    % fps of output video; set lower to slow down replay

% --- Load session ---
sessionFile = fullfile(recordingDir, 'session.mat');
if ~isfile(sessionFile)
    error('replaySession:noSession', 'session.mat not found in %s', recordingDir);
end
loaded  = load(sessionFile);
session = loaded.session;
cfg     = session.cfg;
N       = cfg.N;
nFrames = session.nFrames;

% --- Load processing results if available ---
resultsFile = fullfile(recordingDir, 'results.mat');
hasResults  = isfile(resultsFile);
if hasResults
    res = load(resultsFile);
    res = res.results;
    hasCentroids  = isfield(res, 'blobCentroids');
    hasFullResults = isfield(res, 'nConfirmed');
    fprintf('Loaded results.mat (detect-only: %d | full pipeline: %d).\n', ...
            hasResults, hasFullResults);
else
    hasCentroids   = false;
    hasFullResults = false;
    fprintf('No results.mat found — run processRecording first for detection overlay.\n');
end

% --- Write one video per camera ---
for i = 1:N

    videoOut = fullfile(recordingDir, sprintf('cam%d_replay.avi', i));
    vw = VideoWriter(videoOut);
    vw.FrameRate = playbackFps;
    open(vw);

    fprintf('Writing cam %d / %d  (%d frames) -> %s\n', i, N, nFrames, videoOut);

    for k = 1:nFrames

        frame = imread(fullfile(recordingDir, sprintf('cam%d', i), ...
                                sprintf('frame_%06d.tif', k)));

        t      = session.log.timestamps(i, k);
        syncMs = session.log.syncMs(k);

        if hasFullResults
            infoStr = sprintf('cam%d | t=%.2fs | sync %.1fms | blobs: %d | confirmed: %d | reprErr: %.1fpx', ...
                              i, t, syncMs, res.nBlobs(i,k), res.nConfirmed(k), res.meanReprErr(k));
        elseif hasResults
            infoStr = sprintf('cam%d | t=%.2fs | sync %.1fms | blobs: %d', ...
                              i, t, syncMs, res.nBlobs(i, k));
        else
            infoStr = sprintf('cam%d | t=%.2fs | sync %.1fms', i, t, syncMs);
        end

        % Convert grayscale to RGB so overlays render in colour.
        annotated = repmat(frame, [1 1 3]);

        annotated = insertText(annotated, [10 10], infoStr, ...
                               'FontSize', 14, 'BoxColor', 'black', ...
                               'TextColor', 'white', 'BoxOpacity', 0.6);

        if hasCentroids
            centroids = res.blobCentroids{k}{i};
            if ~isempty(centroids)
                circles   = [centroids, repmat(8, size(centroids, 1), 1)];
                annotated = insertShape(annotated, 'Circle', circles, ...
                                        'Color', 'green', 'LineWidth', 2);
            end
        end

        writeVideo(vw, annotated);

        if mod(k, 100) == 0
            fprintf('  cam %d: %d / %d frames\n', i, k, nFrames);
        end
    end

    close(vw);
    fprintf('  Saved: %s\n', videoOut);
end

fprintf('Done.\n');
