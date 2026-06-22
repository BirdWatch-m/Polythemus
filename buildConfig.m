function cfg = buildConfig()
% BUILDCONFIG Creates the system configuration struct.


cfg.N          = 2;
cfg.camIndices = [2 3];
cfg.resolution = [1280, 720];
cfg.fps        = 30;

cfg.cameraFocus       = 0;
cfg.cameraControlMode = 'focusOnly';
cfg.autoSettleSeconds = 5;
cfg.camProfiles.MY8077 = struct('ExposureControl','auto', ...
    'BacklightCompensation',0, 'Sharpness',0, 'Gamma',300, ...
    'Brightness',0, 'Contrast',64, 'Saturation',128, ...
    'Zoom',0, 'Pan',0, 'Tilt',0, 'Roll',3);
cfg.camProfiles.C922   = struct('ExposureControl','auto', ...
    'BacklightCompensation',0, 'Sharpness',128, ...
    'Brightness',128, 'Contrast',128, 'Saturation',128, 'Zoom',100, 'Pan',0, 'Tilt',0);

cfg.ringBufLen   = 30;
cfg.medianBufLen = 30;

cfg.bgUpdateInterval  = 15;
cfg.medianFgThreshold = 15;
cfg.bgMedianStride    = 2;
cfg.useGMM            = true;

cfg.useMorphology     = false;
cfg.morphKernelRadius = 2;
cfg.morphStrel        = strel('disk', cfg.morphKernelRadius);

cfg.epiThreshold  = 40.0;

cfg.reprThreshold = 15.0;

cfg.minBlobArea = 4;
cfg.maxBlobArea = 6000;

cfg.maxAspect = 6.0;

cfg.maxSyncError = 0.033;

cfg.maxCoastFrames   = 30;
cfg.minConfirmFrames = 3;
cfg.minTrackAge      = 10;

cfg.kalmanProcNoise = 4.0;

cfg.kalmanMeasNoise = 15;

cfg.trackGate = 11.34;

cfg.maxRange = 500;

cfg.kalmanInitVelVar = 625;

cfg.calExtrinsics.captureSeconds   = 5;
cfg.calExtrinsics.surfMetricThresh = 300;
cfg.calExtrinsics.matchThreshold   = 50;
cfg.calExtrinsics.maxRatio         = 0.7;
cfg.calExtrinsics.ransacNumTrials  = 8000;
cfg.calExtrinsics.ransacDistance   = 1.0;
cfg.calExtrinsics.ransacConfidence = 99.99;
cfg.calExtrinsics.minPooledInliers = 100;
cfg.calExtrinsics.maxPoseMatches   = 20000;
cfg.calExtrinsics.fixCam2Coplanar  = false;
cfg.calExtrinsics.fixCam2Level     = true;

cfg.display     = true;
cfg.logDir      = 'output/';
cfg.calFile     = 'calibration/multiCamParams.mat';
cfg.skyMaskFile = 'config/skyMasks.mat';

end
