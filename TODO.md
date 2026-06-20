# Project TODO

Tracked follow-ups that are out of scope for current work but worth doing.

---

## Review gain behaviour after auto-settle  *(low priority)*

Gain was removed from `cfg.camProfiles` (it was pinning both cameras to 0,
suppressing auto-exposure in low light). Now gain floats during the group
settle and locks at whatever the algorithm converges to. Worth verifying
outdoors that the locked gain values are reasonable and not introducing
noise — log them at startup and compare across sessions.

---

## Auto-tune camera parameters for outdoor capture  *(deferred — post first outdoor shoot)*

**Goal:** a helper, e.g. `autoTuneCamera(cam, skyMask, target)`, that sets
exposure/gain automatically at session start (and on demand as light shifts),
instead of hand-tuning per session.

**Design notes (from discussion 2026-06-04):**
- Optimize a **proxy**, not detection rate (no ground-truth birds at tune time):
  target a brightness percentile over the **sky-mask ROI** (e.g. 99th pct ≈ 240),
  minimise clipped-pixel fraction, leave headroom for bright targets.
- **Meter over the sky mask only** — ignore buildings and especially the sun.
- **Exposure = coarse knob** (log₂ steps); **Gain = fine knob** to fill the gaps.
- Application trade-off: **cap exposure *time* for motion blur** on fast birds,
  then use gain to reach the brightness target (accepting noise).
- Drive **both cameras to a common target** so their responses match
  (helps cross-camera association) — not each optimal independently.
- Keep in-camera **contrast/gamma neutral**; do enhancement in software
  (reversible, per-frame). Only auto-tune exposure/gain.
- **Lock** everything after converging (auto drift breaks background subtraction),
  and **rebuild the background model** when re-tuning mid-session.

**Empirical exposure priors — start the search here:**
- **MY8077** (log₂-seconds steps):
  - Clear sky (very bright): **−7 to −11, mostly −9 to −11**.
  - Overcast: shifts somewhat (less negative / brighter).
  - Dusk: shifts *a lot* — a totally different regime.
  - Indoor reference was ≈ −1/−2, so outdoors is ~6–10 stops shorter.
- These are **MY8077-specific**; the **C922 uses a different exposure scale/range**
  and must be characterised separately.

---

## Performance pass — get the live loop to a usable frame rate  *(needed before real bird capture)*

At N=2 / 720p the `main` loop runs **~3 fps** — far too slow to track birds (need
15–30 fps). Acquisition is cheap (~3 ms inter-camera sync); the cost is **detect +
render**. `main.m` now prints per-stage timers (acquire / detect / render ms) — use
those to target the biggest stage first. The culprits are the deferred PERF items:

- **PERF-3 (`renderFrame`):** `insertShape`/`insertText` called *per blob* in a loop,
  each rasterising the whole frame, plus `subplot` every frame. Batch the inserts
  into one array call; reuse axes / use `tiledlayout`. Also consider rendering only
  every Kth frame — display rate need not equal detection rate.
- **PERF-2 (median):** the 60-frame median recompute every `cfg.bgUpdateInterval`
  frames stalls ~0.25 s, and both cameras fire on the same frame. Subsample the
  buffer, stagger the cameras, or switch to a running median (O(1)/pixel, no scan).
- **PERF-1 (double `rgb2gray`):** `updateRingBuf` and `preprocessFrame` both convert
  the same frame. Convert once and pass it through.
- **PERF-5 (sequential snapshots):** lower priority — sync is already ~3 ms at 720p.

Measure with the timers, fix highest-cost first, re-measure.

**Progress (2026-06-05, merged into main):** in-place `renderFrame` (build graphics
once, update CData/markers) took render 116 -> ~16 ms — **the real win**. Median
subsample (`cfg.bgMedianStride`) helped detect somewhat. PERF-1 (single rgb2gray,
threaded through) + int16 median diff turned out to be a **WASH** when finally run:
detect stayed ~90 ms cool, dominated by GMM/median/morphology — rgb2gray is too
cheap to matter and int16 may even be slightly negative (MATLAB optimises double).
Net: **~9 fps cool with GMM on** (was ~3.6); thermal/power throttling erodes it
during a run (all stages slow together). perf/loop is merged. Real remaining
detect levers: the GMM toggle (deferred) or downsampling — not micro-opts.

**Morphology — TEST whether it costs detections (not just perf):**
Now toggled by `cfg.useMorphology`, radius in `cfg.morphKernelRadius`, strel
precomputed in `cfg.morphStrel` (no per-frame rebuild). The risk: `imopen` with
disk-2 erodes a 2px border, so it can **erase ~3x3px distant birds that
`minBlobArea = 9` is meant to keep** — a false-negative source at range. The area
gate already drops single-pixel noise, so `imopen` may be largely redundant.
Test next daylight: `useMorphology = false` (cut it) and/or `morphKernelRadius = 1`,
watch false negatives vs false positives at range.

**Deferred decision — median-only (`cfg.useGMM = false`):** gives ~13 fps but
many more false positives (26-39 blobs on clouds/foliage). The GMM AND suppresses
those. Re-evaluate once the association/tracking layer exists to reject them
downstream — only then can median-only vs GMM be judged fairly.

---

## Depth-uncertainty vs range figure — `diagnostics/depthUncertainty.m`  *(thesis figure; spec'd 2026-06-20)*

**Why.** Triangulation at this rig is low-parallax: `decomposeStability` measured a
**1.22° convergence angle** at the scene's ~34 baseline-lengths median depth
(~102 m at a 3.5 m baseline). That makes *metric depth* intrinsically imprecise —
independent of calibration quality — because depth error scales as range² and
inversely with baseline. This figure quantifies that limit, justifies the
wider airport baselines proposed in the paper, and gives a principled value for
`cfg.kalmanMeasNoise` (whose comment currently assumes an 8 m baseline, ~2.3×
optimistic for the real 3.5 m). Half the paper is experimental; this is the
figure that turns "depth is poor at range" into numbers.

**Core model (closed form, for the curves + intuition).**
- Per-camera centroid localisation error `σ_px` (sweep ~0.5–2 px; a small distant
  bird blob is worse than a checkerboard corner).
- Angular ray error `σ_θ ≈ σ_px / f`, `f ≈ 950 px` at 720p (from intrinsics).
- Parallax angle at range Z, baseline B: `θ ≈ B / Z` (small-angle, frontal).
- **Depth (along-range) σ:** `σ_Z ≈ (Z² / (B · f)) · σ_px`  ⇒ `σ_Z / Z ≈ σ_px /(f·θ)`.
- **Lateral (cross-range) σ:** `σ_lat ≈ (Z / f) · σ_px` — grows only linearly, so
  the uncertainty ellipsoid is cigar-shaped along the line of sight. Report both;
  the anisotropy is the point (and motivates a non-isotropic, range-dependent
  Kalman R — see ASSUMPTIONS.md, currently isotropic constant by choice).

**Rigorous version (validate the closed form).**
- Build the linearised triangulation covariance: stack the per-camera projection
  Jacobians `J_i = ∂(proj)/∂X` at the true point, form the information matrix
  `Σ_X = (Σ_i J_iᵀ J_i / σ_px²)^{-1}`, report its eigenvalues/vectors (major axis ≈
  depth direction). Generalises cleanly to N≥3 cameras (just sum more J terms).
- **Monte-Carlo cross-check:** project a known point into each camera with the
  real calibration, add `N(0, σ_px²)` pixel noise, triangulate via the pipeline's
  own `triangulateGroups`, repeat ~1e4×, compare empirical scatter to `Σ_X`.
  They should match — this also exercises the real triangulation path, not a
  textbook stand-in.

**Configurations to plot.**
- Baselines: **3.5 m** (current test rig), **2.25 m** and **6.5 m** (planned
  intra-/inter-balcony, per CLAUDE.md), plus 1–2 airport-scale values (e.g.
  20 m, 50 m) for the proposal section.
- N: the N=2 pair vs an N=3 layout (show the third camera's differently-oriented
  baseline shrinks the depth axis — the real argument for 3 cameras beyond
  disambiguation).
- Range axis: 20–150 m operating band shaded; extend to ~500 m for airport.

**Deliverables.**
1. `σ_Z` (m) vs range, one curve per baseline, operating band shaded (log-y ok).
2. `σ_Z / Z` (%) vs range — the "how good is a distance estimate" plot.
3. Top-down error-ellipse cartoon at 50/100/150 m showing the cigar shape (cross
   `σ_lat` vs along `σ_Z`).
4. Small table: `σ_Z` at 50/100/150 m per baseline, and the `kalmanMeasNoise`
   (m²) each implies at a representative range.
- Use the real intrinsics for `f`; take geometry (convergence) from the actual
  calibration where possible rather than assuming frontal-parallel.

**Follow-on once plotted:** set `cfg.kalmanMeasNoise` from `σ_Z` at a representative
range (and fix its 8 m-baseline comment), and reconsider whether the isotropic
measurement-noise assumption is costing tracking accuracy at long range.

---

## Retire `salvageCalibration.m`  *(cleanup; flagged 2026-06-20)*

It still carries the original convention bug twice (`R_k = relOri'`, and the
file-load transpose `R_fixed = old.R{2}'`), and its premise — transpose whatever
R sits in the file — is only valid if that R came from the broken SURF path, so
it is unsound in general. The proper path is now `calibrateExtrinsics` (pooled,
convention-correct) or `calibrateExtrinsicsCheckerboard`. Delete `salvageCalibration.m`
(and the stale `calibration/multiCamParams_salvaged.mat`) once you've confirmed
you no longer depend on either.
