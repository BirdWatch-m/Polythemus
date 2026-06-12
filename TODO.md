# Project TODO

Tracked follow-ups that are out of scope for current work but worth doing.

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
