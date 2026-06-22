function analyzeResults(recordingDir, varargin)
% ANALYZERESULTS  Detection-reliability and reconstruction-consistency report.
%
%   analyzeResults(recordingDir, Name, Value, ...) reads ONE processed recording
%   (session.mat + results.mat) and produces the Chapter 6 evidence for that
%   recording: a pipeline-funnel detection-reliability figure, a track-continuity
%   summary, and a reprojection-error distribution. It runs only on data already
%   on disk; it does not re-run the pipeline.
%
%   PROVENANCE (printed and stamped on every figure, so validity is never
%   ambiguous): the recording folder, its frame window, and the exact detection
%   config and extrinsic baseline that produced these results. Extrinsics differ
%   per session, so the baseline shown is read from THIS recording's own
%   calibration snapshot.
%
%   NAME-VALUE OPTIONS
%     'resultsFileName'  'results.mat' (default) or 'results_tuned.mat'
%     'frameRange'       [first last] absolute frames where the target is present,
%                        so detection rates are conditioned on a target being up
%                        rather than diluted by empty-sky frames. [] = all frames.
%     'outDir'           where to write outputs ('' -> next to the recording)
%
%   Outputs, written to outDir:
%     analyze_detection_funnel.png   per-stage survival + continuity
%     analyze_reproj_hist.png        reprojection-error distribution
%     analyze_summary.txt            the printed report
%
%   See also: processRecording, replayTracks3D, triangulateGroups

opt.resultsFileName = 'results.mat';
opt.frameRange      = [];
opt.outDir          = '';
opt = parseOpts(opt, varargin);

% =========================================================================
% LOAD
% =========================================================================
sessionFile = fullfile(recordingDir, 'session.mat');
resultsFile = fullfile(recordingDir, opt.resultsFileName);
assert(isfile(sessionFile), 'analyzeResults:noSession', 'session.mat not found in %s', recordingDir);
assert(isfile(resultsFile), 'analyzeResults:noResults', '%s not found. Run processRecording first.', resultsFile);

S = load(sessionFile); session = S.session; cfg = session.cfg;
R = load(resultsFile); results = R.results;
outDir = opt.outDir; if isempty(outDir), outDir = recordingDir; end
if ~isfolder(outDir), mkdir(outDir); end

% Absolute frame index per result column.
if isfield(results, 'frameIndex'), frameIdx = results.frameIndex(:).';
else, frameIdx = 1:size(results.nBlobs, 2); end

% Restrict to the target-present window.
if isempty(opt.frameRange), keep = true(size(frameIdx));
else
    assert(numel(opt.frameRange) == 2 && opt.frameRange(1) <= opt.frameRange(2), ...
           'analyzeResults:badRange', 'frameRange must be [] or [first last].');
    keep = frameIdx >= opt.frameRange(1) & frameIdx <= opt.frameRange(2);
end
nWin = nnz(keep);
assert(nWin > 0, 'analyzeResults:emptyWindow', 'No result frames fall in frameRange.');

hasFull = isfield(results, 'nConfirmed');
baseline = NaN;
if isfield(session, 'calibration') && isfield(session.calibration, 't') && numel(session.calibration.t) >= 2
    baseline = norm(session.calibration.t{2});
end
fps = cfg.fps;

% =========================================================================
% PROVENANCE BANNER  (the validity check)
% =========================================================================
cfgLine1 = sprintf('medianThr=%g  GMM=%d  morph=%d  minArea=%g  maxArea=%g  aspect=%g', ...
    cfg.medianFgThreshold, cfg.useGMM, cfg.useMorphology, cfg.minBlobArea, cfg.maxBlobArea, cfg.maxAspect);
cfgLine2 = sprintf('epiThr=%gpx  reprThr=%gpx  baseline=%.2fm  %dx%d@%gfps', ...
    cfg.epiThreshold, cfg.reprThreshold, baseline, cfg.resolution(1), cfg.resolution(2), fps);
[~, recTag] = fileparts(recordingDir);
if isempty(opt.frameRange), winStr = sprintf('all %d frames', nWin);
else, winStr = sprintf('frames %d-%d (%d)', opt.frameRange(1), opt.frameRange(2), nWin); end

rep = {};
rep{end+1} = sprintf('=== analyzeResults: %s / %s ===', recTag, opt.resultsFileName);
rep{end+1} = sprintf('Window : %s   (%.1f s)', winStr, nWin / fps);
rep{end+1} = sprintf('Config : %s', cfgLine1);
rep{end+1} = sprintf('         %s', cfgLine2);
if ~hasFull
    rep{end+1} = 'NOTE: detect-only results (no association/triangulation/tracking fields).';
end

% =========================================================================
% DETECTION FUNNEL  (% of windowed frames surviving each pipeline stage)
% =========================================================================
nB   = results.nBlobs(:, keep);
c1   = 100 * mean(nB(1,:) >= 1);
c2   = 100 * mean(nB(2,:) >= 1);
both = 100 * mean(all(nB >= 1, 1));        % both cameras see something (needed to triangulate)

stageNames = {'cam1 blob', 'cam2 blob', 'both cams'};
stagePct   = [c1, c2, both];

if hasFull
    grp  = 100 * mean(results.nGroups(keep)  >= 1);
    pts  = 100 * mean(results.nPoints(keep)  >= 1);
    conf = 100 * mean(results.nConfirmed(keep) >= 1);
    stageNames = [stageNames, {'assoc group', 'valid 3D pt', 'confirmed track'}];
    stagePct   = [stagePct,   grp, pts, conf];
end

rep{end+1} = '';
rep{end+1} = 'Detection funnel (% of windowed frames):';
for s = 1:numel(stageNames)
    rep{end+1} = sprintf('  %-16s %6.1f %%', stageNames{s}, stagePct(s)); %#ok<SAGROW>
end

% =========================================================================
% TRACK CONTINUITY  (fragmentation: how many distinct confirmed tracks, how long)
% =========================================================================
contLines = {};
if hasFull && isfield(results, 'trackIds')
    tIds = results.trackIds(keep);
    allIds = [];
    for k = 1:numel(tIds), allIds = [allIds; tIds{k}(:)]; end %#ok<AGROW>
    uids = unique(allIds(~isnan(allIds)));
    nActive = zeros(size(uids)); spanSec = zeros(size(uids));
    for j = 1:numel(uids)
        present = cellfun(@(c) any(c(:) == uids(j)), tIds);
        idx = find(present);
        nActive(j) = numel(idx);
        spanSec(j) = (idx(end) - idx(1) + 1) / fps;
    end
    if isempty(uids)
        contLines{end+1} = 'Track continuity: no confirmed tracks in window.';
    else
        contLines{end+1} = sprintf('Track continuity: %d distinct confirmed track(s)', numel(uids));
        contLines{end+1} = sprintf('  active frames/track : median %.0f, max %d', median(nActive), max(nActive));
        contLines{end+1} = sprintf('  span/track (s)      : median %.2f, max %.2f', median(spanSec), max(spanSec));
    end
    rep = [rep, {''}, contLines];
end

% =========================================================================
% REPROJECTION CONSISTENCY  (internal-consistency reconstruction quality)
% =========================================================================
reproj = [];
if hasFull
    e = results.meanReprErr(keep);
    reproj = e(~isnan(e));
    if ~isempty(reproj)
        rep{end+1} = '';
        rep{end+1} = sprintf('Reprojection error (per-frame mean, %d valid frames):', numel(reproj));
        rep{end+1} = sprintf('  median %.2f px | p90 %.2f px | max %.2f px | gate %.1f px', ...
            median(reproj), prctile(reproj, 90), max(reproj), cfg.reprThreshold);
    end
end

% --- print + save the report ---
txt = strjoin(rep, newline);
fprintf('%s\n', txt);
fid = fopen(fullfile(outDir, 'analyze_summary.txt'), 'w');
if fid > 0, fprintf(fid, '%s\n', txt); fclose(fid); end

% =========================================================================
% FIGURE 1: detection funnel + provenance + continuity
% =========================================================================
bannerFig = sprintf('median=%g GMM=%d morph=%d minA=%g  |  epi=%gpx repr=%gpx base=%.2fm  |  %dx%d@%gfps', ...
    cfg.medianFgThreshold, cfg.useGMM, cfg.useMorphology, cfg.minBlobArea, ...
    cfg.epiThreshold, cfg.reprThreshold, baseline, cfg.resolution(1), cfg.resolution(2), fps);

f1 = newFig(8.4, 4.8);
ax = axes(f1); hold(ax, 'on'); set(ax, 'FontName', 'Arial', 'FontSize', 11);
bar(ax, stagePct, 0.6, 'FaceColor', [0.30 0.50 0.74], 'EdgeColor', 'none');
set(ax, 'XTick', 1:numel(stageNames), 'XTickLabel', stageNames, 'XTickLabelRotation', 15);
ylim(ax, [0 max(100, max(stagePct) * 1.15)]); ylabel(ax, '% of windowed frames'); grid(ax, 'on'); box(ax, 'on');
for s = 1:numel(stagePct)
    text(ax, s, stagePct(s) + 2, sprintf('%.1f%%', stagePct(s)), ...
         'HorizontalAlignment', 'center', 'FontSize', 9);
end
title(ax, sprintf('Detection reliability  -  %s  (%s)', recTag, winStr), 'FontWeight', 'bold', 'Interpreter', 'none');
subtitle(ax, bannerFig, 'FontSize', 8.5, 'Color', [0.35 0.35 0.35], 'Interpreter', 'none');
if ~isempty(contLines)
    text(ax, 0.02, 0.97, strjoin(contLines, newline), 'Units', 'normalized', ...
         'VerticalAlignment', 'top', 'FontName', 'Arial', 'FontSize', 8.5, 'Color', [0.2 0.2 0.2], ...
         'BackgroundColor', [1 1 1], 'EdgeColor', [0.8 0.8 0.8], 'Margin', 3, 'Interpreter', 'none');
end
exportgraphics(f1, fullfile(outDir, 'analyze_detection_funnel.png'), 'Resolution', 300); close(f1);

% =========================================================================
% FIGURE 2: reprojection-error distribution
% =========================================================================
if ~isempty(reproj)
    f2 = newFig(7.0, 4.6);
    ax = axes(f2); hold(ax, 'on'); set(ax, 'FontName', 'Arial', 'FontSize', 11);
    histogram(ax, reproj, 'BinWidth', 0.1, 'FaceColor', [0.30 0.50 0.74], 'EdgeColor', 'none');
    xline(ax, median(reproj), '-', sprintf('median %.2f px', median(reproj)), 'Color', [0.1 0.1 0.1], 'LineWidth', 1.4);
    xline(ax, cfg.reprThreshold, '--', sprintf('gate %.1f px', cfg.reprThreshold), 'Color', [0.7 0.2 0.2], 'LineWidth', 1.2);
    grid(ax, 'on'); box(ax, 'on');
    xlabel(ax, 'Per-frame mean reprojection error (px)'); ylabel(ax, 'Frames');
    title(ax, sprintf('Reconstruction internal consistency  -  %s', recTag), 'FontWeight', 'bold', 'Interpreter', 'none');
    exportgraphics(f2, fullfile(outDir, 'analyze_reproj_hist.png'), 'Resolution', 300); close(f2);
end

fprintf('\nSaved figures + summary to %s\n', outDir);

end

% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================
function fig = newFig(w, h)
fig = figure('Color', 'w', 'Units', 'inches', 'Position', [1 1 w h], 'Visible', 'off');
try, theme(fig, 'light'); catch, end
set(fig, 'Color', 'w');
end

function opt = parseOpts(opt, args)
for k = 1:2:numel(args)
    name = args{k};
    if isfield(opt, name), opt.(name) = args{k+1};
    else, error('analyzeResults:badOpt', 'Unknown option "%s".', name); end
end
end
