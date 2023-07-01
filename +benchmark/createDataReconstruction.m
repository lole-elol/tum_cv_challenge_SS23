%% Script to generate benchmark config files

% Output File
outPath = '+benchmark/configs/benchmarkReconstructionV1.mat'

reconstruction = struct(...
    'log', false, ... % Preprocessing:
    'scalingFactor', 0.5, ... % Feature Extraction:
    'numOctaves', 20, ...
    'roiBorder', 20, ... % Epipolar Geometry:
    'eMaxDistance', [0.5, 5, 50], ...
    'eConfidence', 99.6, ...
    'eMaxNumTrials', [1000, 100000], ...
    'eValidPointFraction', 0.8, ... % Triangulation:
    'maxReprojectionError', 20 ...
)
%% Save to file
save(outPath, 'reconstruction')

% %% Run benchmark