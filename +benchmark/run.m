function run(configPath, inPath, outPath, varargin)
% BENCHMARK - Run 3D Reconstruction and 3D Model Detection benchmarks with different parameters
%
% Inputs:
%   configPath: Path to the configuration file
%   outPath: Path to the output folder
%   inPath: Path(s) to the input folder(s) (Includes images or point clouds depending on the benchmark)
%   testReconstruction = true: If true, run the 3D Reconstruction benchmark
%   testDetection = true: If true, run the 3D Model Detection benchmark
%   numImages = Inf: Number of images to use for the 3D Reconstruction benchmark
%   maxNumCombinationsToLog = 10: Maximum number of parameter combinations to log in the console

isArrayOrString = @(x) isstring(x) || iscell(x);

p = inputParser;
p.addRequired("configPath", @isstring)
p.addRequired("inPath", isArrayOrString)
p.addRequired("outPath", @isstring)
p.addParameter('testReconstruction', true, @islogical);
p.addParameter('testDetection', true, @islogical);
p.addParameter('numImages', Inf, @isnumeric);
p.addParameter('maxNumCombinationsToLog', 10, @isnumeric);

p.parse(configPath, inPath, outPath, varargin{:});

testReconstruction = p.Results.testReconstruction;
testDetection = p.Results.testDetection;
numImages = p.Results.numImages;
maxNumCombinationsToLog = p.Results.maxNumCombinationsToLog;

disp('Running Benchmark!')
disp('Loading configurations .mat file');
% Load configuration from .mat files (i.e. the structs reconstruction and detection)
load(configPath);

if testReconstruction
    disp('Running 3D Reconstruction benchmark');
    % Load input data
    scenes = cell(length(inPath), 1);
    if ~iscell(inPath)
        inPath = {inPath};
    end
    for i = 1:length(inPath)
        path = inPath{i};
        images = util.loadImages(path + '/images', numImages=numImages);
        cameraParams = logic.reconstruct3D.loadCameraParams(path + '/cameras.txt');
        scenes{i} = {images, cameraParams};
        fprintf("\n");
    end
    numScenes = length(scenes);

    % Prepare all the combinations of parameters
    fields = fieldnames(reconstruction);
    numParams = numel(fields);

    combinationsArgs = cell(1, numel(fields));
    for i = 1:numel(fields)
        combinationsArgs{i} = reconstruction.(fields{i});
    end

    reconstructionParams = combinations(combinationsArgs{:});  % Table containing all combinations
    reconstructionParams.Properties.VariableNames = fields;
    numCombinations = size(reconstructionParams, 1);

    % Save parameter combinations
    outputMat = append(outPath, '/reconstructionParams.mat');
    save(outputMat, 'reconstructionParams');

    disp('Found ' + string(numCombinations) + ' parameter combinations:');
    if numCombinations > maxNumCombinationsToLog
        disp(reconstructionParams(1:maxNumCombinationsToLog, :))
        disp('...')
    else
        disp(reconstructionParams)
    end
    input('Press enter to continue ...');
    disp('Running benchmark ...');
    fprintf('\n')

    % Run the benchmark by running the reconstruction algorithm with each
    % parameter combination and each scene. Save the output data.
    outData = cell(numCombinations, numScenes);
    testPCs = cell(numCombinations, 1);  % PC that will be passed to the detection benchmarking
    for i=1:numCombinations
        paramsTable = reconstructionParams(i, :);
        disp('Running parameter combination ' + string(i) + ' of ' + string(numCombinations));
        fprintf('\n')
        disp(paramsTable)
        fprintf('\n')

        params = util.getParamsFromTable(paramsTable);

        for j = 1:numScenes
            try
                scene = scenes{j};
                images = scene{1};
                cameraParams = scene{2};
                disp('Running scene ' + string(j) + ' of ' + string(numScenes));
                tic
                [testPCs{i}, camPoses, tracks] = logic.reconstruct3DMultiview(images, cameraParams, params{:});
                t = toc;
                outData{i, j} = {testPCs{i}, camPoses, tracks, t};
                disp('Time: ' + string(t) + 's');
            catch
                disp('Error running scene ' + string(j) + ' of ' + string(numScenes));
                outData{i, j} = "Error";
            end
        end
    end
    % Save output data
    save(append(outPath, '/reconstructionOut.mat'), 'outData');
    disp('3D Reconstruction benchmark done!')
end

if testDetection
    disp('Running 3D Model Detection benchmark');
    if ~testReconstruction
        % Load input data, since it has not been computed before
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
    numTestPCs = size(testPCs, 1);
    % Prepare all the combinations of parameters
    detectionFields = fieldnames(detection);
    combinationsArgs = cell(1, numel(detectionFields));
    for i = 1:numel(detectionFields)
        combinationsArgs{i} = detection.(detectionFields{i});
    end
    detectionParams = combinations(combinationsArgs{:});  % Table containing all combinations
    detectionParams.Properties.VariableNames = detectionFields;
    numCombinations = size(detectionParams, 1);
    % Save parameter combinations
    outputMat = append(outPath, '/detectionParams.mat');
    save(outputMat, 'detectionParams');

    disp('Found ' + string(numCombinations) + ' parameter combinations:');
    if numCombinations > maxNumCombinationsToLog
        disp(detectionParams(1:maxNumCombinationsToLog, :))
        disp('...')
    else
        disp(detectionParams)
    end
    input('Press enter to continue ...');
    disp('Running benchmark ...');
    fprintf('\n')

    outData = cell(numCombinations, numTestPCs);
    for i=1:size(detectionParams, 1)
        paramsTable = detectionParams(i, :);

        disp('Running parameter combination ' + string(i) + ' of ' + string(numCombinations));
        fprintf('\n')
        disp(paramsTable)
        fprintf('\n')

        params = util.getParamsFromTable(paramsTable);

        for j=1:numTestPCs
            pc = testPCs{j};
            disp('Running point cloud ' + string(j) + ' of ' + string(numTestPCs));

            tic
            [models, pcFilter, pcRemaining] = logic.modelDetection(pc, params{:});
            t = toc;

            outData{i, j} = {models, pcFilter, pcRemaining, t};

            disp('Time: ' + string(t) + 's');
        end
        fprintf('\n')
    end

    % Save output data
    save(append(outPath, '/detectionOut.mat'), 'outData');
    disp('3D Model Detection benchmark done!')
end

disp('Benchmark completed!')
end