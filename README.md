# BirdTracker

Multi-camera bird detection and 3D tracking system for a Bachelor's final project in Aerospace Engineering. The system uses 2–3 stationary cameras to detect small flying objects (birds, drones) against the sky and triangulate their 3D positions in real time.

## Project Status

**Multi-camera detection pipeline working at N=2 on live sky (720p).**

| Module | Status |
|---|---|
| Configuration | ✅ Complete |
| Camera acquisition | ✅ Complete |
| Ring buffer | ✅ Complete |
| Sky mask drawing | ✅ Complete |
| Background subtraction (median + GMM) | ✅ Complete (per-camera) |
| Blob extraction and gating | ✅ Complete |
| Live display | ✅ Complete |
| Session logging | ✅ Complete |
| Multi-camera detection loop (`main.m`) | ✅ Runs at N=2; ~9.4 fps after PERF pass |
| Intrinsic calibration | ✅ Complete (MY8077 + C922, 1080p & 720p) |
| Extrinsic calibration | ⚠️ Run once at N=2, not yet validated |
| Cross-camera association | ❌ Not yet implemented |
| Multi-view triangulation | ❌ Not yet implemented |
| 3D Kalman tracking | ❌ Not yet implemented |
| Offline replay | ⚠️ Partial |

## Project Structure

```
birdtracker/
├── buildConfig.m              All system parameters
├── initSystem.m               System initialisation (multi-camera entry point)
├── main.m                     Multi-camera detection driver / smoke test (N cameras)
├── testSingleCamera.m         Smoke test — runs the full single-camera pipeline
├── replaySession.m            Generate annotated video from saved frames + log
│
├── acquisition/
│   ├── acquireFrames.m
│   ├── updateRingBuf.m
│   └── syncCheck.m
│
├── detection/
│   ├── detectBlobs.m
│   ├── preprocessFrame.m
│   ├── applyBackground.m
│   └── gateBlobs.m
│
├── io/
│   ├── renderFrame.m
│   ├── logFrame.m
│   └── saveSession.m
│
├── calibration/
│   ├── calibrateIntrinsics.m
│   ├── calibrateExtrinsics.m
│   ├── validateCalibration.m
│   └── CALIBRATION_EXPLAINED.txt
│
└── config/
    └── drawSkyMasks.m
```

## Requirements

- MATLAB R2021b or later
- Image Processing Toolbox
- Computer Vision Toolbox
- MATLAB Support Package for USB Webcams

## Quick Start

1. Add the project folder to the MATLAB path (run from the project root):
   ```matlab
   addpath(genpath(pwd))
   ```
2. Set `cfg.N` and `cfg.camIndices` in `buildConfig.m` to match your rig — run `webcamlist()` to find the physical indices (the order is OS-driven; the laptop built-in is usually index 1).
3. Calibrate camera intrinsics (one-time per camera), at the resolution you will operate at:
   ```matlab
   % Edit camIdx, saveName, and the camera-property block in calibrateIntrinsics.m, then run.
   calibrateIntrinsics
   ```
4. Single-camera smoke test:
   ```matlab
   testSingleCamera
   ```
5. Multi-camera detection (full-frame masks; optional extrinsic calibration up front):
   ```matlab
   main
   ```

The single-camera smoke test prompts for a sky mask if none exists, then runs the pipeline and prints a diagnostic report. `main` runs the N-camera detection loop with per-stage timing; press **Q** to stop.

## Hardware

The system is designed for 2–3 USB webcams (Logitech C920 class) connected to a host laptop. Cameras must have:
- Manual exposure / focus lock capability
- USB Video Class (UVC) compatibility
- Minimum 720p resolution at 30fps

The integrated laptop webcam works for testing the pipeline but is too slow (~5fps with snapshot timeout issues) for production use.

**USB bandwidth:** two 1080p webcams cannot stream simultaneously unless they sit on *separate* USB host controllers — isochronous bandwidth reservation fails. On a shared controller the system runs at **720p**. Check your USB topology (Device Manager → "Devices by connection") if you need 1080p from both cameras.

## License

[Add when ready]
