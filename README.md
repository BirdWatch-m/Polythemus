# BirdTracker

Multi-camera bird detection and 3D tracking system for a Bachelor's final project in Aerospace Engineering. The system uses 2–3 stationary cameras to detect small flying objects (birds, drones) against the sky and triangulate their 3D positions in real time.

## Project Status

**Baseline — single-camera detection pipeline working.**

| Module | Status |
|---|---|
| Configuration | ✅ Complete |
| Camera acquisition | ✅ Complete |
| Ring buffer | ✅ Complete |
| Sky mask drawing | ✅ Complete |
| Background subtraction (median + GMM) | ✅ Complete |
| Blob extraction and gating | ✅ Complete |
| Live display | ✅ Complete |
| Session logging | ✅ Complete |
| Intrinsic calibration | ✅ Complete |
| Extrinsic calibration | ⚠️ Code written, untested (requires multiple cameras) |
| Cross-camera association | ❌ Not yet implemented |
| Multi-view triangulation | ❌ Not yet implemented |
| 3D Kalman tracking | ❌ Not yet implemented |
| Offline replay | ⚠️ Partial |

## Project Structure

```
birdtracker/
├── buildConfig.m              All system parameters
├── initSystem.m               System initialisation (multi-camera entry point)
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

1. Add the project folder to the MATLAB path:
   ```matlab
   addpath(genpath('birdtracker'))
   ```
2. Calibrate camera intrinsics (one-time, per camera):
   ```matlab
   % Edit camIdx and saveFile in calibrateIntrinsics.m, then run.
   calibrateIntrinsics
   ```
3. Run the single-camera smoke test:
   ```matlab
   testSingleCamera
   ```

The smoke test will prompt you to draw a sky mask if one does not exist, then run the detection pipeline for 30 seconds and print a diagnostic report.

## Hardware

The system is designed for 2–3 USB webcams (Logitech C920 class) connected to a host laptop. Cameras must have:
- Manual exposure / focus lock capability
- USB Video Class (UVC) compatibility
- Minimum 720p resolution at 30fps

The integrated laptop webcam works for testing the pipeline but is too slow (~5fps with snapshot timeout issues) for production use.

## License

[Add when ready]
