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
