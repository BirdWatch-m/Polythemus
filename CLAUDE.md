# Project Context — BirdTracker

## What this is

Bachelor's final project in Aerospace Engineering at Politehnica University of Bucharest. Multi-camera bird/drone detection and 3D tracking system. The paper will be 60-70 pages — roughly half theoretical (camera physics, image processing algorithms, trilateration), half experimental (real birds tracked from a 5th floor balcony with 2-3 webcams). The paper also proposes a large-scale airport implementation.

## Implementation

- **Language:** MATLAB R2021b+
- **Toolboxes:** Image Processing, Computer Vision, MATLAB Support Package for USB Webcams
- **Hardware target:** 2-3 Logitech C920-class webcams + host laptop
- **Repo:** github.com/BirdWatch-m/Polythemus (private)

## Architecture

Modular pipeline, designed for arbitrary N cameras from the start (not 2-only with N=3 retrofitted):

```
Frame acquisition (timestamp-aligned, ring-buffered)
  -> Per-camera preprocessing (sky mask + temporal median + GMM, ANDed; morphology; blob gating)
  -> Cross-camera association (epipolar gate via fundamental matrices, Hungarian assignment, partial observations allowed)
  -> Multi-view triangulation (DLT least-squares + reprojection gate)
  -> 3D Kalman tracking (constant-velocity, lifecycle states tentative/confirmed/coasting)
  -> Output / logging
```

## File layout

```
buildConfig.m            All system parameters (single source of truth)
initSystem.m             System init for the multi-camera entry point
testSingleCamera.m       Smoke test — runs full single-camera pipeline with diagnostics
replaySession.m          Generate annotated video offline from raw frames + log

acquisition/             acquireFrames, updateRingBuf, syncCheck
detection/               detectBlobs, preprocessFrame, applyBackground, gateBlobs
io/                      renderFrame, logFrame, saveSession
calibration/             calibrateIntrinsics, calibrateExtrinsics, validateCalibration
config/                  drawSkyMasks
```

## Status

| Module | Status |
|---|---|
| Configuration, acquisition, ring buffer | Done |
| Sky mask drawing | Done |
| Background subtraction (median + GMM) | Done |
| Blob extraction and gating | Done |
| Live display, logging, session save | Done |
| Intrinsic calibration | Done, tested on laptop webcam |
| Extrinsic calibration | Code written, untested (needs multiple cameras) |
| Cross-camera association | Designed, not implemented |
| Multi-view triangulation | Designed, not implemented |
| 3D Kalman tracking | Designed, not implemented |
| Offline replay | Partial |

## Coding conventions for this project

- **Function header docs:** comprehensive; describe inputs/outputs, behaviour, errors, and "see also" references
- **Inline comments:** maximum 1-2 short lines per comment block, preferably shorter; explain *what* the code does, not *why* the field exists (that goes in headers)
- **No magic numbers** anywhere outside `buildConfig.m`
- **Long explanations** of algorithms or concepts go in separate `.txt` files (see `calibration/CALIBRATION_EXPLAINED.txt` as a template)
- **No globals.** All state lives in caller-passed structs.
- **Cell arrays** for per-camera collections (`cams{i}`, `ringBuf{i}`, `blobs{i}`)
- **Function signatures preferred over inline scripts** except for top-level entry points (testSingleCamera, replaySession) where inline user inputs are easier to edit

## Hardware constraints (real, not theoretical)

- 5th floor apartment balconies, ~10m apart, facing NW, in Bucharest
- Planned: C1, C2 on balcony A (2.25m apart), C3 on balcony B (6.5m from C2)
- Cameras will be USB webcams (no hardware sync, ~5ms timestamp jitter assumed)
- Cables run through apartment interior, runs >10m may need USB-over-Cat6 extenders
- Balconies are enclosed in glass — reflections to be handled by mounting strategy
- Birds: swallows (~19cm body, 15-22m/s, erratic), pigeons (35cm, 10-20m/s), crows, seagulls, small songbirds
- Range: 20m to 150m practical

## What the user prefers

- Concise technical language, no descriptive adjectives where not required
- Lists assumptions made
- Asks clarifying questions to resolve ambiguity rather than guessing
- Sufficiently complex math/concepts should be explained in *slightly* simpler language but not dumbed down
- Line-by-line code explanation when introducing new concepts
- Code updates: prefer showing me exactly what to change (find/replace style) rather than rewriting whole files when changes are small
- For larger changes or new files, write them in full
