# Assumptions & Design Decisions

Baked-in assumptions and design choices in the multi-view pipeline
(association → triangulation → tracking), recorded so they are explicit and
revisitable. Each notes the trade-off and what would change it. Add to this as
modules grow.

---

## System-wide

- **World frame = camera 1.** All 3D geometry is in camera 1's frame
  (`R{1}=I`, `t{1}=0`), set by `calibrateExtrinsics`.
- **Manual focus everywhere** — every camera-open site calls `applyCameraSettings`,
  which locks focus to `cfg.cameraFocus` (the SAME value `calibrateIntrinsics`
  uses). Autofocus would change the effective focal length and silently
  invalidate the intrinsic calibration. Exposure and white balance are
  auto-converged to the current scene then locked to manual (drift would break
  background subtraction). Per-model structural settings come from
  `cfg.camProfiles`, keyed by `cam.Name`.
- **Pose convention: world-to-camera** — `X_cam = R·X_world + t`, so projection
  is `P = K·[R|t]` and `F{i,j} = K_j^-T [t_rel]_x R_rel K_i^-1`.
  **Verified numerically** (`tests/testExtrinsicConventions.m`). The two toolbox
  helpers do NOT share a convention: `extrinsics()` returns a postmultiply R, so
  premultiply needs a transpose — `calibrateExtrinsicsCheckerboard` uses
  `R12 = Rb2'*Rb1`. `relativeCameraPose()` returns a camera-POSE orientation that
  already equals the premultiply R, so it must NOT be transposed —
  `calibrateExtrinsics` uses `R_rel = relOri` (an earlier `relOri'` was a ~17°
  rotation bug, since corrected).
- **Extrinsics: cam2 forward offset forced to 0** (`cfg.calExtrinsics.fixCam2Coplanar`).
  The baseline component along cam1's optical axis is near-unobservable from a
  distant scene (low parallax: ~1.2° convergence at ~100m), so it drifts ~±1m
  run-to-run while X/Y stay tight; it is pinned to 0, true to a few cm for the
  level, ~0-toe-in side-by-side 2-cam rig. VALID only while cameras are
  near-coplanar — invalid for the planned cross-balcony C3 (genuinely staggered
  in depth), which needs near-field parallax or a measured offset. Evidence in
  `diagnostics/extrinsicsStability`.
- **Camera model:** pinhole + radial/tangential distortion from the intrinsics
  objects; image points are undistorted before triangulation.
- **Synchronous frames:** all cameras' detections in a frame are treated as
  simultaneous. Timestamp sync is handled upstream (`syncCheck`), not here.
- **Units:** pixels for image-space gates (`epiThreshold`, `reprThreshold`),
  metres for 3D (`trackGate`, Kalman noise).

## associateViews — cross-camera association

- **Epipolar distance = symmetric point-to-line** (mean of both directions), not
  the Sampson distance. Cheaper; swap to Sampson if accuracy demands.
- **One-to-one per camera pair** via Hungarian (`matchpairs`), gated at
  `cfg.epiThreshold`; `costUnmatched = epiThreshold`.
- **N≥3 grouping = connected components** of pairwise matches (union-find).
  *Exact for N=2* (current rig). For N≥3, an inconsistent set of pairwise
  matches could place two blobs from one camera in a single group; the
  downstream reprojection gate is relied on to reject those. A rigorous global
  multi-view assignment is deferred.
- **Partial observations kept:** a blob seen in only one camera survives as a
  singleton group (`nViews=1`) for the tracker.
- **Convention-agnostic:** uses only `F` and the epipolar relation, independent
  of the extrinsic pose convention.

## triangulateGroups — multi-view triangulation

- **Linear DLT** (minimises algebraic error via SVD null space), not a
  geometric/reprojection-optimal or iterative method. Standard and fast; a
  nonlinear refinement would be marginally more accurate.
- **≥2 views required;** groups with fewer are skipped.
- **Reprojection gate** (`cfg.reprThreshold`) computed in undistorted pixel
  space; points over threshold are KEPT but flagged `valid=false` (diagnostics),
  not dropped.
- **2-view blind spot:** a mismatch *along* the epipolar line is geometrically
  consistent (rays intersect at the wrong depth) and passes BOTH the epipolar
  and reprojection gates. Needs a 3rd camera or temporal tracking to catch.
- **K** is read as `intrinsics.IntrinsicMatrix'` (MATLAB's transposed convention).

## updateTracks — 3D Kalman tracking

- **Constant-velocity motion model**, 6-state `[px py pz vx vy vz]`,
  measurement = 3D position. **Hand-rolled** Kalman filter (not `trackingKF`),
  for transparency and unit-testability.
- **Process noise = white random acceleration**, isotropic, scaled by
  `cfg.kalmanProcNoise`. Erratic manoeuvres (swallows) are modelled *only* as
  process noise, not a manoeuvring-target model.
- **Measurement noise isotropic** `R = kalmanMeasNoise·I` — the same uncertainty
  for every point regardless of range or geometry, even though triangulation is
  genuinely less certain at long range / narrow baseline.
- **New track:** zero initial velocity, large initial velocity variance
  (`cfg.kalmanInitVelVar`).
- **Association = Euclidean gated nearest-neighbour** via Hungarian
  (`matchpairs`), gate `cfg.trackGate` (metres). Not Mahalanobis (which would use
  the innovation covariance — more principled, deferred).
- **Confirmation on CUMULATIVE matched frames** ≥ `cfg.minConfirmFrames` (not
  strictly consecutive, despite the cfg comment wording).
- **Lifecycle:** tentative → confirmed (on confirm count); a tentative track that
  misses once is dropped immediately; a confirmed track coasts up to
  `cfg.maxCoastFrames` missed frames, then is deleted.
- **One-to-one** measurement↔track per frame; no track splitting, merging, or
  multi-hypothesis tracking.
- **`dt` passed per frame;** assumes a known (roughly uniform) time step.
