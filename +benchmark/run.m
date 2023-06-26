function run(configPath, inPath, outPath, varargin)
% BENCHMARK - Run 3D Reconstruction and 3D Model Detection benchmarks with different parameters
%
% Inputs:
%   configPath: Path to the configuration file
%   outPath: Path to the output folder
%   inPath: Path to the input folder (Includes images or point clouds depending on the benchmark)
%   testReconstruction = true: If true, run the 3D Reconstruction benchmark
%   testDetection = true: If true, run the 3D Model Detection benchmark

p = inputParser;
p.addParameter('testReconstruction', true, @islogical);
p.addParameter('testDetection', true, @islogical);

p.parse(varargin{:});
testReconstruction = p.Results.testReconstruction;
testDetection = p.Results.testDetection;

disp('Running Benchmark!')
disp('Loading configuration .mat file');
load(configPath);

if testReconstruction
    disp('Running 3D Reconstruction benchmark');
    disp('! NOT IMPLEMENTED YET !')
end

if testDetection
    disp('Running 3D Model Detection benchmark');
    if ~testReconstruction
        % Load input data
        disp('Loading input point clouds');
        files = dir(append(inPath, '/*.txt'));

        if isempty(files)
            error('No point clouds found in the input folder');
        end

        testPCs = cell(length(files), 1);
        for i=1:length(files)
            el = files(i);
            testPCs{i} = logic.pointcloud.loadData(append(inPath, '/', el.name));
        end
    end

    % Generate all possible parameter combinations
    outlierDist = detection.outlierDist;
    clusterDist = detection.clusterDist;
    clusterPercentile = detection.clusterPercentile;
    clusterDenoise = detection.clusterDenoise;
    clusterDenoiseNeighbours = detection.clusterDenoiseNeighbours;
    ceilingPercentile = detection.ceilingPercentile;
    ceilingDist = detection.ceilingDist;
    ceilingWindowSize = detection.ceilingWindowSize;
    cuboidVolume = detection.cuboidVolume;
    cuboidInlier = detection.cuboidInlier;
    cuboidOverlap = detection.cuboidOverlap;

    detectionParams = combinations(outlierDist, clusterDist, clusterPercentile, clusterDenoise, clusterDenoiseNeighbours, ceilingPercentile, ceilingDist, ceilingWindowSize, cuboidVolume, cuboidInlier, cuboidOverlap);

    % Save parameter combinations
    save(append(outPath, '/detectionParams.mat'), 'detectionParams');

    disp('Found ' + string(size(detectionParams, 1)) + ' parameter combinations');
    disp('Running benchmark ...');
    fprintf('\n')

    outData = cell(size(detectionParams, 1), size(testPCs, 1));
    for i=1:size(detectionParams, 1)
        params = detectionParams(i, :);

        disp('Running parameter combination ' + string(i));
        disp('----------------------------------------');
        disp('outlierDist: ' + string(params.outlierDist));
        disp('clusterDist: ' + string(params.clusterDist));
        disp('clusterPercentile: ' + string(params.clusterPercentile));
        disp('clusterDenoise: ' + string(params.clusterDenoise));
        disp('clusterDenoiseNeighbours: ' + string(params.clusterDenoiseNeighbours));
        disp('ceilingPercentile: ' + string(params.ceilingPercentile));
        disp('ceilingDist: ' + string(params.ceilingDist));
        disp('ceilingWindowSize: ' + string(params.ceilingWindowSize));
        disp('cuboidVolume: ' + string(params.cuboidVolume));
        disp('cuboidInlier: ' + string(params.cuboidInlier));
        disp('cuboidOverlap: ' + string(params.cuboidOverlap));
        disp('----------------------------------------')
        fprintf('\n')

        for j=1:length(testPCs)
            pc = testPCs{j};
            disp('Running point cloud ' + string(j) + ' of ' + string(size(testPCs, 1)));

            [models, pcFilter, pcRemaining] = logic.modelDetection(pc, outlierDist=params.outlierDist, clusterDist=params.clusterDist, clusterPercentile=params.clusterPercentile, clusterDenoise=params.clusterDenoise, clusterDenoiseNeighbours=params.clusterDenoiseNeighbours, ceilingPercentile=params.ceilingPercentile, ceilingDist=params.ceilingDist, ceilingWindowSize=params.ceilingWindowSize, cuboidVolume=params.cuboidVolume, cuboidInlier=params.cuboidInlier, cuboidOverlap=params.cuboidOverlap);

            outData{i, j} = {models, pcFilter, pcRemaining};

        end

        fprintf('\n')
    end

    % Save output data
    save(append(outPath, '/detectionOut.mat'), 'outData');
    disp('3D Model Detection benchmark done!')
end

disp('Benchmark completed!')
end