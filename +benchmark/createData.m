%% Script to generate benchmark config files

% Output File
% outPath = 'config/presortBenchmark.mat'
outPath = '+benchmark/configs/benchmarkPresort.mat'


%% 3D Model Reconstruction Parameters
reconstruction = struct(...
    'log', false, ... % Preprocessing:
    'scalingFactor', 0.1, ... % Feature Extraction:
    'numOctaves', 20,...
    'roiBorder', 20, ... % Epipolar Geometry:
    'eMaxDistance', 5, ...
    'eConfidence', 99.6, ...
    'eMaxNumTrials',  100000, ...
    'eValidPointFraction', 0.8, ... % Triangulation:
    'maxReprojectionError', 20, ...
    'presort', ["FFT2", "HIST", "PCA"], ...
    'presortNearestNeighbors', [true, false], ...
    'presortFeatures', [1, 10], ...
    'presortNormalize', [true, false] ...
)

%% 3D Model Detection Parameters
outlierDist = 3;
clusterDist = 0.09;
clusterPercentile = 0.0005;
clusterDenoise = 0.4;
clusterDenoiseNeighbours = 50;
ceilingPercentile = 0.2;
ceilingDist = 0.3;
ceilingWindowSize = 3;
cuboidVolume = 0.1;
cuboidInlier = 3.5;
cuboidOverlap = 0.8;

detection = struct('outlierDist', outlierDist, 'clusterDist', clusterDist, 'clusterPercentile', clusterPercentile, 'clusterDenoise', clusterDenoise, 'clusterDenoiseNeighbours', clusterDenoiseNeighbours, 'ceilingPercentile', ceilingPercentile, 'ceilingDist', ceilingDist, 'ceilingWindowSize', ceilingWindowSize, 'cuboidVolume', cuboidVolume, 'cuboidInlier', cuboidInlier, 'cuboidOverlap', cuboidOverlap)

%% Save to file
save(outPath, 'detection', 'reconstruction')