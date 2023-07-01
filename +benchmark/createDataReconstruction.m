%% Script to generate benchmark config files

% Output File
outPath = '+benchmark/configs/benchmarkReconstructionV1.mat'

reconstruction = struct(...
    'log', false, ... % Preprocessing:
    'scalingFactor', 0.5, ... % Feature Extraction:
    'numOctaves', 20, ...
    'roiBorder', 20, ... % Epipolar Geometry:
    'eMaxDistance', [5, 6], ...
    'eConfidence', 99.6, ...
    'eMaxNumTrials', 100000, ...
    'eValidPointFraction', 0.8, ... % Triangulation:
    'maxReprojectionError', 20 ...
)
%% Save to file
save(outPath, 'reconstruction')

% %% Run benchmark
% benchmark.run("+benchmark/configs/benchmarkReconstructionV1.mat", {"test/old_computer", "test/delivery_area_dslr_undistorted"}, "+benchmark/outputData", testDetection=false, testReconstruction=true)
% benchmark.run({"+benchmark/configs/benchmarkReconstructionV1.mat", "+benchmark/configs/benchmarkCuboids.mat"}, {"test/old_computer", "test/delivery_area_dslr_undistorted"}, "+benchmark/outputData", testDetection=true, testReconstruction=true)