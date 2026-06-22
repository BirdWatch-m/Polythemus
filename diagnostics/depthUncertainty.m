function out = depthUncertainty(varargin)
% DEPTHUNCERTAINTY  Triangulation depth-uncertainty versus range for the network.
%
%   out = depthUncertainty(Name, Value, ...) computes and plots how the 3D
%   reconstruction uncertainty of the camera network grows with target range,
%   for several baselines. It produces the thesis figures that quantify the
%   low-parallax depth limit: depth error scales as range^2 and inversely with
%   baseline, so metric depth is intrinsically imprecise at range regardless of
%   calibration quality. This is geometry, not a code defect.
%
%   The depth (along-range) and lateral (cross-range) standard deviations are
%   reported from three independent routes that should agree:
%     1. CLOSED FORM   sigma_Z   ~= Z^2 * sigma_px / (B * f)   (Ch7 eq.),
%                      sigma_lat ~= Z   * sigma_px /  f.
%     2. LINEARISED    Sigma_X = ( sum_i J_i' J_i / sigma_px^2 )^-1, with J_i the
%                      projection Jacobian of camera i at the true point. Handles
%                      N>=2 and real (non-frontal) geometry; its eigenvectors give
%                      the uncertainty ellipsoid axes.
%     3. MONTE CARLO   project a known point into each camera with the real
%                      calibration, add N(0, sigma_px^2) pixel noise, triangulate
%                      with the pipeline's own triangulateGroups, repeat, and take
%                      the empirical covariance.
%   Routes 2 and 3 propagate both noisy views, so they sit a factor ~sqrt(2)
%   above the first-order closed form; that gap is the honest two-view cost and
%   is annotated on the validation figure. The rigorous (route 2) value at the
%   representative range is what the tracker measurement noise should use.
%
%   OUTPUTS (also saved as PNG at 300 DPI + a .mat in opt.outDir)
%     out.range, out.sigZ [nB x nR], out.relErr [nB x nR]  closed-form families
%     out.table          struct array: per-baseline sigma_Z at table ranges and
%                        the isotropic kalmanMeasNoise (m^2) implied at repRange
%     out.validation     closed/linearised/MC sigma_Z at the primary baseline
%
%   Geometry note: the closed-form families assume a frontal-parallel rig with
%   the real mean focal length. The linearised and Monte-Carlo validation use the
%   actual recorded calibration (calibration/multiCamParams.mat), so the
%   agreement also confirms the real rig matches the idealised model. Ideal
%   pinhole projection is used for the Monte-Carlo (the lenses are corrected to
%   0.17-0.20 px, so undistortion is ~identity); this affects the mean negligibly
%   and the covariance not at all.
%
%   See also: triangulateGroups, decomposeStability, buildConfig

% ---------------------------- USER-EDITABLE DEFAULTS ----------------------------
opt.baselines   = [2.25 3.5 6.5 20 50];      % m. 3.5 = current rig; 2.25/6.5 = planned
                                             % intra-/inter-balcony; 20/50 = airport scale.
opt.primaryBase = 3.5;                       % m. Baseline for the ribbon, ellipses, validation.
opt.sigmaPx     = 1.0;                        % px. Representative per-camera centroid error.
opt.sigmaPxBand = [0.5 2.0];                  % px. Ribbon around the primary-baseline curve.
opt.rangeM      = 10:2:500;                   % m. Range axis.
opt.opBand      = [20 150];                   % m. Operating band (shaded).
opt.tableRanges = [50 100 150];               % m. Ranges for the ellipse cartoon and table.
opt.repRange    = 100;                        % m. Range at which kalmanMeasNoise is implied.
opt.nMonteCarlo = 1e4;                         % Monte-Carlo trials per validation range.
opt.threeCam    = [0 2.25 8.75];              % m. Planned 3-camera lateral layout (collinear span).
opt.calFile     = 'calibration/multiCamParams.mat';
opt.intr1File   = 'calibration/intrinsics_MY1_720.mat';
opt.intr2File   = 'calibration/intrinsics_LG1_720.mat';
opt.outDir      = 'output/diagnostics/depthUncertainty';
% --------------------------------------------------------------------------------
opt = parseOpts(opt, varargin);

projRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projRoot, 'triangulation'));   % triangulateGroups (Monte-Carlo path)
resolve = @(p) char(fullfile(projRoot, p));
if ~isfolder(resolve(opt.outDir)), mkdir(resolve(opt.outDir)); end

% Real focal length: mean fx over the two cameras.
K1 = loadK(resolve(opt.intr1File));
K2 = loadK(resolve(opt.intr2File));
f  = mean([K1(1,1) K2(1,1)]);
fprintf('depthUncertainty: f = %.1f px (MY8077 %.1f, C922 %.1f), sigma_px = %.2f px\n', ...
        f, K1(1,1), K2(1,1), opt.sigmaPx);

% =============================================================================
% 1. CLOSED-FORM FAMILIES
% =============================================================================
Z  = opt.rangeM;
nB = numel(opt.baselines);
sigZ   = zeros(nB, numel(Z));
relErr = zeros(nB, numel(Z));
for b = 1:nB
    [sigZ(b,:), ~, relErr(b,:)] = closedForm(Z, opt.baselines(b), f, opt.sigmaPx);
end
[sigZlo, ~, relLo] = closedForm(Z, opt.primaryBase, f, opt.sigmaPxBand(1));
[sigZhi, ~, relHi] = closedForm(Z, opt.primaryBase, f, opt.sigmaPxBand(2));

% =============================================================================
% 2. LINEARISED + MONTE-CARLO VALIDATION (real rig, primary baseline)
% =============================================================================
cal = load(resolve(opt.calFile));  cal = cal.multiCamParams;
cfg.N = numel(cal.R);  cfg.reprThreshold = Inf;
Kcell = cell(1, cfg.N);
for i = 1:cfg.N, Kcell{i} = cal.intrinsics{i}.IntrinsicMatrix.'; end

valRanges = opt.tableRanges;
val.closed = zeros(1, numel(valRanges));
val.lin    = zeros(1, numel(valRanges));
val.mc     = zeros(1, numel(valRanges));
fprintf('\nValidation at the real %.2f m rig (sigma_Z, m):\n', norm(cal.t{2}));
fprintf('  %5s  %8s  %10s  %9s\n', 'Z(m)', 'closed', 'linearised', 'MonteC');
for r = 1:numel(valRanges)
    X = [0; 0; valRanges(r)];                          % on camera-1 optical axis
    val.closed(r) = closedForm(valRanges(r), norm(cal.t{2}), f, opt.sigmaPx);
    Sig           = linCov(cal.R, cal.t, Kcell, X, opt.sigmaPx);
    val.lin(r)    = sqrt(Sig(3,3));
    empSig        = mcTriangulate(cal, cfg, X, opt.sigmaPx, opt.nMonteCarlo);
    val.mc(r)     = sqrt(empSig(3,3));
    fprintf('  %5.0f  %8.2f  %10.2f  %9.2f\n', valRanges(r), val.closed(r), val.lin(r), val.mc(r));
end

% Linearised sigma_Z over the full range axis (primary 2-camera rig + planned 3-camera).
Rfront = {eye(3), eye(3)};
cen2   = {[0;0;0], [opt.primaryBase;0;0]};
cen3   = cell(1, numel(opt.threeCam));
for ii = 1:numel(opt.threeCam), cen3{ii} = [opt.threeCam(ii); 0; 0]; end
Rfr3   = repmat({eye(3)}, 1, numel(opt.threeCam));
linZ2  = arrayfun(@(z) szFrontal(cen2, Rfront, f, [0; 0; z],                  opt.sigmaPx), Z);
linZ3  = arrayfun(@(z) szFrontal(cen3, Rfr3,   f, [mean(opt.threeCam); 0; z], opt.sigmaPx), Z);

% =============================================================================
% 3. UNCERTAINTY TABLE
% =============================================================================
tbl = struct('baseline', {}, 'sigZ', {}, 'kalmanMeasNoise', {});
fprintf('\nsigma_Z (m) by baseline and range (closed form, sigma_px=%.2f):\n', opt.sigmaPx);
hdr = sprintf('  %8s', 'B (m)');
for r = 1:numel(opt.tableRanges), hdr = [hdr sprintf('  %6gm', opt.tableRanges(r))]; end %#ok<AGROW>
fprintf('%s   %s\n', hdr, sprintf('R@%g (m^2)', opt.repRange));
for b = 1:nB
    sZt  = closedForm(opt.tableRanges, opt.baselines(b), f, opt.sigmaPx);
    R100 = closedForm(opt.repRange, opt.baselines(b), f, opt.sigmaPx)^2;
    tbl(b) = struct('baseline', opt.baselines(b), 'sigZ', sZt, 'kalmanMeasNoise', R100);
    fprintf('  %8.2f', opt.baselines(b));
    fprintf('  %7.2f', sZt);
    fprintf('   %10.2f\n', R100);
end
fprintf('(Rigorous two-view sigma_Z is ~sqrt(2)x larger; R@100 from route 2 = %.2f m^2 at B=%.2f.)\n', ...
        val.lin(valRanges==opt.repRange)^2, opt.primaryBase);

% =============================================================================
% FIGURES
% =============================================================================
cmap = lines(nB);
stem = resolve(fullfile(opt.outDir, 'depthUncertainty'));

% --- Fig 1: sigma_Z vs range, one curve per baseline ---
f1 = newFig(7.2, 4.7);
ax = axes(f1); hold(ax,'on'); set(ax,'FontName','Arial','FontSize',11);
fill(ax,[Z fliplr(Z)],[sigZhi fliplr(sigZlo)],0.80*[1 1 1], ...
     'EdgeColor','none','FaceAlpha',0.45,'DisplayName',sprintf('\\sigma_{px} %.1f-%.1f px @ %.2f m',opt.sigmaPxBand,opt.primaryBase));
h = gobjects(1,nB);
for b = 1:nB
    h(b) = plot(ax, Z, sigZ(b,:), 'LineWidth', 2, 'Color', cmap(b,:), ...
                'DisplayName', sprintf('B = %.2f m', opt.baselines(b)));
end
set(ax,'YScale','log'); grid(ax,'on'); box(ax,'on');
xline(ax, opt.opBand(1), ':', 'Color',[.4 .4 .4]); xline(ax, opt.opBand(2), ':', 'Color',[.4 .4 .4]);
xlabel(ax,'Target range Z (m)'); ylabel(ax,'Depth uncertainty \sigma_Z (m)');
title(ax, sprintf('Depth uncertainty vs range  (f = %.0f px, \\sigma_{px} = %.1f px)', f, opt.sigmaPx), ...
      'FontWeight','bold');
legend(ax,[h fill_handle(ax)],'Location','northwest','FontSize',9);
xlim(ax,[Z(1) Z(end)]);
exportgraphics(f1,[stem '_sigmaZ_vs_range.png'],'Resolution',300); close(f1);

% --- Fig 2: sigma_Z / Z (%) vs range ---
f2 = newFig(7.2, 4.7);
ax = axes(f2); hold(ax,'on'); set(ax,'FontName','Arial','FontSize',11);
fill(ax,[Z fliplr(Z)],[relHi fliplr(relLo)],0.80*[1 1 1],'EdgeColor','none','FaceAlpha',0.45);
for b = 1:nB
    plot(ax, Z, relErr(b,:), 'LineWidth', 2, 'Color', cmap(b,:), ...
         'DisplayName', sprintf('B = %.2f m', opt.baselines(b)));
end
grid(ax,'on'); box(ax,'on');
xline(ax, opt.opBand(1), ':', 'Color',[.4 .4 .4]); xline(ax, opt.opBand(2), ':', 'Color',[.4 .4 .4]);
yline(ax, 10, '--', '10 %', 'Color',[.3 .3 .3], 'LabelHorizontalAlignment','left');
xlabel(ax,'Target range Z (m)'); ylabel(ax,'Relative depth uncertainty \sigma_Z / Z (%)');
title(ax,'Fractional depth uncertainty vs range','FontWeight','bold');
opMask = Z >= opt.opBand(1) & Z <= opt.opBand(2);
legend(ax,'Location','northwest','FontSize',9); xlim(ax,[Z(1) Z(end)]);
ylim(ax,[0 min(60, max(relErr(:,opMask),[],'all')*1.8)]);
exportgraphics(f2,[stem '_relErr_vs_range.png'],'Resolution',300); close(f2);

% --- Fig 3: error ellipses at table ranges (primary baseline), one zoomed panel
%     each so the true sigma_lat : sigma_Z elongation (the cigar) is visible. ---
f3 = newFig(9.4, 3.7);
tl = tiledlayout(f3, 1, numel(opt.tableRanges), 'TileSpacing','compact','Padding','compact');
th = linspace(0, 2*pi, 160);
for r = 1:numel(opt.tableRanges)
    Zr = opt.tableRanges(r);
    [sZr, sLat] = closedForm(Zr, opt.primaryBase, f, opt.sigmaPx);
    ax = nexttile(tl); hold(ax,'on'); set(ax,'FontName','Arial','FontSize',10); axis(ax,'equal');
    fill(ax, sLat*cos(th), sZr*sin(th), cmap(2,:), 'FaceAlpha',0.18, 'EdgeColor',cmap(2,:),'LineWidth',1.6);
    plot(ax, [0 0], [-sZr sZr], '-', 'Color',cmap(2,:),'LineWidth',0.6);
    plot(ax, [-sLat sLat], [0 0], '-', 'Color',cmap(2,:),'LineWidth',0.6);
    plot(ax, 0, 0, '+', 'Color',cmap(2,:),'MarkerSize',9,'LineWidth',1.2);
    lim = 1.25*sZr; xlim(ax,[-lim lim]); ylim(ax,[-lim lim]);
    grid(ax,'on'); box(ax,'on');
    title(ax, sprintf('Z = %g m', Zr), 'FontWeight','bold');
    xlabel(ax,'cross-range (m)');
    if r == 1, ylabel(ax,'along line of sight (m)'); end
    text(ax, 0, -lim*0.92, sprintf('\\sigma_{lat} = %.2f m\n\\sigma_Z = %.2f m   (%.0f:1)', sLat, sZr, sZr/sLat), ...
         'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize',9);
end
title(tl, sprintf('Reconstruction uncertainty ellipses, B = %.2f m  (1\\sigma; long axis = line of sight)', ...
      opt.primaryBase), 'FontWeight','bold');
exportgraphics(f3,[stem '_ellipses.png'],'Resolution',300); close(f3);

% --- Fig 4: validation (closed vs linearised vs Monte-Carlo) ---
f4 = newFig(7.2, 4.7);
ax = axes(f4); hold(ax,'on'); set(ax,'FontName','Arial','FontSize',11);
plot(ax, Z, linZ2, '-',  'LineWidth',2, 'Color',cmap(2,:), 'DisplayName','linearised, N=2 (B=3.5 m)');
plot(ax, Z, linZ3, '-',  'LineWidth',2, 'Color',cmap(3,:), 'DisplayName',sprintf('linearised, N=3 (span %.2f m)',opt.threeCam(end)));
[cf35,~] = closedForm(Z, opt.primaryBase, f, opt.sigmaPx);
plot(ax, Z, cf35, '--', 'LineWidth',1.6, 'Color',[.35 .35 .35], 'DisplayName','closed form, N=2 (first order)');
plot(ax, valRanges, val.mc, 'o', 'MarkerSize',8, 'MarkerFaceColor',cmap(2,:), ...
     'MarkerEdgeColor','k', 'DisplayName','Monte-Carlo (real rig)');
set(ax,'YScale','log'); grid(ax,'on'); box(ax,'on');
xlabel(ax,'Target range Z (m)'); ylabel(ax,'Depth uncertainty \sigma_Z (m)');
title(ax,'Validation: closed form vs linearised vs Monte-Carlo','FontWeight','bold');
legend(ax,'Location','northwest','FontSize',9); xlim(ax,[Z(1) min(200,Z(end))]);
exportgraphics(f4,[stem '_validation.png'],'Resolution',300); close(f4);

% =============================================================================
% SAVE
% =============================================================================
out.f = f; out.sigmaPx = opt.sigmaPx; out.range = Z;
out.baselines = opt.baselines; out.sigZ = sigZ; out.relErr = relErr;
out.table = tbl; out.validation = val; out.linZ2 = linZ2; out.linZ3 = linZ3;
save([stem '_data.mat'], 'out', 'opt');
fprintf('\nSaved 4 figures + data to %s\n', resolve(opt.outDir));

end

% ================================= HELPERS =================================
function [sZ, sLat, rel] = closedForm(Z, B, f, sPx)
sZ   = (Z.^2 .* sPx) ./ (B .* f);
sLat = (Z   .* sPx) ./ f;
rel  = 100 .* sZ ./ Z;
end

function K = loadK(file)
S = load(file); fn = fieldnames(S); o = S.(fn{1});
K = o.IntrinsicMatrix.';
end

function Sig = linCov(Rcell, tcell, Kcell, X, sPx)
% Linearised triangulation covariance from real world-to-camera R, t.
N = numel(Rcell); Lambda = zeros(3);
for i = 1:N
    P = Kcell{i} * [Rcell{i}, tcell{i}(:)];
    J = numJac(P, X);
    Lambda = Lambda + (J.' * J) / sPx^2;
end
Sig = inv(Lambda);
end

function Sig = linCovFrontal(centers, Rcell, f, X, sPx)
% Same as linCov but for a synthetic frontal rig given camera CENTRES and a
% single scalar focal length (principal point cancels in the Jacobian).
N = numel(centers); K = [f 0 0; 0 f 0; 0 0 1]; Lambda = zeros(3);
for i = 1:N
    R = Rcell{i}; t = -R * centers{i}(:);
    J = numJac(K*[R t], X);
    Lambda = Lambda + (J.' * J) / sPx^2;
end
Sig = inv(Lambda);
end

function sZ = szFrontal(centers, Rcell, f, X, sPx)
% Depth (along-range, world Z) standard deviation for a synthetic frontal rig.
Sig = linCovFrontal(centers, Rcell, f, X, sPx);
sZ  = sqrt(Sig(3,3));
end

function J = numJac(P, X)
d = 1e-4; J = zeros(2,3);
for k = 1:3
    Xp = X(:); Xp(k) = Xp(k) + d;
    Xm = X(:); Xm(k) = Xm(k) - d;
    J(:,k) = (proj(P, Xp) - proj(P, Xm)) / (2*d);   % central difference
end
end

function x = proj(P, X)
xh = P * [X(:); 1]; x = xh(1:2) / xh(3);
end

function empSig = mcTriangulate(cal, cfg, X, sPx, nTrials)
N = cfg.N; pts0 = zeros(N, 2);
for i = 1:N
    P = (cal.intrinsics{i}.IntrinsicMatrix.') * [cal.R{i}, cal.t{i}(:)];
    pts0(i,:) = proj(P, X);
end
P3 = zeros(nTrials, 3);
for m = 1:nTrials
    grp.camIds = 1:N;
    grp.points = pts0 + sPx * randn(N, 2);
    pt = triangulateGroups(grp, cal, cfg);
    P3(m,:) = pt(1).position;
end
empSig = cov(P3);
end

function h = fill_handle(ax)
% Return the patch (ribbon) handle so it can be appended to the legend.
h = findobj(ax, 'Type', 'patch'); if isempty(h), h = gobjects(0); else, h = h(end); end
end

function opt = parseOpts(opt, args)
for k = 1:2:numel(args)
    name = args{k};
    if isfield(opt, name), opt.(name) = args{k+1};
    else, error('depthUncertainty:badOpt', 'Unknown option "%s".', name); end
end
end

function fig = newFig(w, h)
% White figure with a forced light theme (R2025b figures otherwise follow the
% desktop theme, which can render axes dark in -batch).
fig = figure('Color','w','Units','inches','Position',[1 1 w h],'Visible','off');
try, theme(fig, 'light'); catch, end
set(fig, 'Color', 'w');
end
