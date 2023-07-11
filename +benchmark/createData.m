%% Script to generate benchmark config files

% Output File
outPath = 'config/paramsV2.mat'
%outPath = '+benchmark/configs/benchmarkEpipolar.mat'


%% 3D Model Reconstruction Parameters
reconstruction = struct(...
    'log', false, ... % Preprocessing:
    'scalingFactor', 0.5, ... % Feature Extraction:
    'numOctaves', 3,...
    'roiBorder', 2, ... % Epipolar Geometry:
    'eMaxDistance', 2, ...
    'eConfidence', 99.9900, ...
    'eMaxNumTrials',  9000, ...
    'eValidPointFraction', 0.75, ... % Triangulation:
    'maxReprojectionError', 8, ...
    'presort', "HIST", ...
    'presortFeatures', 1, ...
    'presortLazy', false ...
)

%% 3D Model Detection Parameters
detection = struct(...
    'outlierDist', 3, ...
    'clusterDist', 0.09, ...
    'clusterPercentile', 0.0005, ...
    'clusterDenoise', 0.4, ...
    'clusterDenoiseNeighbours', 50, ...
    'ceilingPercentile', 0.2, ...
    'ceilingDist', 0.3, ...
    'ceilingWindowSize', 3, ...
    'floorDist', 0.3, ...
    'cuboidVolume', 0.1, ...
    'cuboidInlier', 3.5, ...
    'cuboidOverlap', 0.8 ...
)


%% Save to file
save(outPath, 'detection', 'reconstruction')