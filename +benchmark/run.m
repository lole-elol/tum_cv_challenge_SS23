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
disp('')
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
    paramComb = combinations(detection.outlierDist, detection.clusterDist, detection.clusterPercentile, detection.clusterDenoise, detection.clusterDenoiseNeighbours, detection.ceilingPercentile, detection.ceilingDist, detection.ceilingWindowSize, detection.cuboidVolume, detection.cuboidInlier, detection.cuboidOverlap);

    % Save parameter combinations
    save(append(outPath, '/paramComb.mat'), 'paramComb');

    disp('Found ' + string(length(paramComb)) + ' parameter combinations');
    disp('Running benchmark ...');

    outData = cell(length(paramComb), length(testPCs));
    for i=1:size(paramComb, 1)
        params = paramComb(i, :);

        disp('Running parameter combination ' + string(i));
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
        disp('')

        for j=1:length(testPCs)
            pc = testPCs{j};
            disp('Running point cloud ' + string(j) + ' of ' + string(length(testPCs)));

            [models, pcFilter, pcRemaining] = logic.modelDetection(pc, outlierDist=params.outlierDist, clusterDist=params.clusterDist, clusterPercentile=params.clusterPercentile, clusterDenoise=params.clusterDenoise, clusterDenoiseNeighbours=params.clusterDenoiseNeighbours, ceilingPercentile=params.ceilingPercentile, ceilingDist=params.ceilingDist, ceilingWindowSize=params.ceilingWindowSize, cuboidVolume=params.cuboidVolume, cuboidInlier=params.cuboidInlier, cuboidOverlap=params.cuboidOverlap);

            outData{i, j} = struct('models', models, 'pcFilter', pcFilter, 'pcRemaining', pcRemaining);
        end
    end

    % Save output data
    save(append(outPath, '/detectionOut.mat'), 'outData');
end


end