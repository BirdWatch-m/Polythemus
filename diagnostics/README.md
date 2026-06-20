# diagnostics/

Read-only diagnostic scripts. None modify calibration or config; each prints a
report (a few also save to `output/diagnostics/`). Run from the project root
with `addpath(genpath(pwd))` first.

## Calibration / extrinsics

Typical order when bringing up or debugging an extrinsic calibration:

| Script | Question it answers |
|---|---|
| `inspectIntrinsics` | Are the per-camera intrinsics good? (reprojection error, distortion footprint, resolution match) — one-time sanity. |
| `epipolarOnRecording` | **Acceptance test.** Does a calibration's epipolar geometry match real scene points? Target < 1 px median. Compares to a scene-fit floor, so it separates "calibration wrong" from "hardware can't do better". |
| `extrinsicsStability` | Is the result repeatable? Reports cam2 centre [X,Y,Z] scatter over N runs, proves the forward (Z) axis is unobservable, and shows `fixCam2Coplanar` removing that wobble. |
| `montecarloExtrinsicsPooled` | How many pooled frames are needed for stability? Sweeps frame count; shows the single-snapshot lottery collapsing to one mode. |
| `decomposeStability` | Root-cause split: is the fundamental matrix unstable, or only the F→pose decomposition (low parallax)? Also reports the parallax/convergence angle. |

Convention is regression-tested separately in `tests/testExtrinsicConventions.m`
(world-to-camera premultiply; `relativeCameraPose` orientation used un-transposed).

## Camera characterisation (pre-existing)

| Script | Purpose |
|---|---|
| `sweepC922Sharpness`, `mapC922Sharpness`, `runC922BlurDiagnostics`, `sweepC922Control` | C922 sharpness / blur / control characterisation. |
| `tuneMY8077ExposureGain` | MY8077 exposure & gain tuning. |
