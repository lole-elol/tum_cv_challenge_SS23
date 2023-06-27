%% Script to generate benchmark config files

% Output File
outPath = 'config/paramsV1.mat'

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
save(outPath, 'detection')