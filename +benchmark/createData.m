%% Script to generate benchmark config files

% Output File
outPath = '+benchmark/configs/benchmarkV1.mat'

%% 3D Model Detection Parameters
outlierDist = 3;
clusterDist = [0.1, 0.07];
clusterPercentile = 0.0006;
clusterDenoise = 0.5;
clusterDenoiseNeighbours = 10;
ceilingPercentile = 0.2;
ceilingDist = 0.2;
ceilingWindowSize = 5;
cuboidVolume = 0.1;
cuboidInlier = 3;
cuboidOverlap = 0.9;

detection = struct('outlierDist', outlierDist, 'clusterDist', clusterDist, 'clusterPercentile', clusterPercentile, 'clusterDenoise', clusterDenoise, 'clusterDenoiseNeighbours', clusterDenoiseNeighbours, 'ceilingPercentile', ceilingPercentile, 'ceilingDist', ceilingDist, 'ceilingWindowSize', ceilingWindowSize, 'cuboidVolume', cuboidVolume, 'cuboidInlier', cuboidInlier, 'cuboidOverlap', cuboidOverlap)

%% Save to file
save(outPath, 'detection')