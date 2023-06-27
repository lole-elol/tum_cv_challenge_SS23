%% Script to generate benchmark config files

% Output File
outPath = '+benchmark/configs/benchmarkV1.mat'

%% 3D Model Detection Parameters
outlierDist = 3;
clusterDist = [0.12, 0.1, 0.07, 0.05];
clusterPercentile = [0.008, 0.0006, 0.0004];
clusterDenoise = [1, 0.8, 0.5, 0.3];
clusterDenoiseNeighbours = [10, 50, 100];
ceilingPercentile = [0.3, 0.2, 0.1];
ceilingDist = [0.3, 0.2, 0.1];
ceilingWindowSize = [4, 5, 8];
cuboidVolume = [0, 0.1, 0.2];
cuboidInlier = [3.5, 3, 2.5];
cuboidOverlap = 0.9;

detection = struct('outlierDist', outlierDist, 'clusterDist', clusterDist, 'clusterPercentile', clusterPercentile, 'clusterDenoise', clusterDenoise, 'clusterDenoiseNeighbours', clusterDenoiseNeighbours, 'ceilingPercentile', ceilingPercentile, 'ceilingDist', ceilingDist, 'ceilingWindowSize', ceilingWindowSize, 'cuboidVolume', cuboidVolume, 'cuboidInlier', cuboidInlier, 'cuboidOverlap', cuboidOverlap)

%% Save to file
save(outPath, 'detection')