function display(dataPath, varargin)
% DISPLAY Display the benchmark results.
%
% Inputs:
%   dataPath: Path to the output folder of the benchmark.
%   showReconstruction = true: If true, show the results of the 3d reconstruction
%   showDetection = true: If true, show the results of the 3d model detection

p = inputParser;
p.addParameter('showReconstruction', true, @islogical);
p.addParameter('showDetection', true, @islogical);

p.parse(varargin{:});
showReconstruction = p.Results.showReconstruction;
showDetection = p.Results.showDetection;

disp('Displaying Benchmark Results!')

if showReconstruction
    disp('Displaying 3D Reconstruction Results!')
    disp("Loading Benchmark Results...")
    load(fullfile(dataPath, 'reconstructionParams.mat'), 'reconstructionParams');
    load(fullfile(dataPath, 'reconstructionOut.mat'), 'outData');
    fprintf('\n')

    numCombinations = size(reconstructionParams, 1);
    for i=1:numCombinations
        params = reconstructionParams(i, :);

        fprintf('Parameter combination %d:\n', i)
        disp(params)
        fprintf('\n')

        figure('name', 'Results for parameter combination ' + string(i));

        for j=1:size(outData, 2)
            subplot(ceil(size(outData, 2)/2), 2, j)
            title('Point Cloud ' + string(j))
            results = outData{i, j};

            % Check if cell is "Error" instead of ceall array with results
            if strcmp(results, 'Error')
                disp("Point Cloud " + string(j) + " could not be reconstructed.")
                continue
            end

            pointCloudInstance = results{1};
            pcFilter = logic.pointcloud.filter(pointCloudInstance, 3);
            disp("Point Cloud " + string(j) + " has " + string(pcFilter.Count) + " points.")
            camPoses = results{2};
            plotting.plotPointCloud(pcFilter, camPoses)
        end
        input('Press any key to continue...')
    end
end

if showDetection
    disp('Displaying 3D Model Detection Results!')
    disp("Loading Benchmark Results...")
    load(fullfile(dataPath, 'detectionParams.mat'), 'detectionParams');
    load(fullfile(dataPath, 'detectionOut.mat'), 'outData');
    fprintf('\n')

    for i=1:size(detectionParams, 1)
        params = detectionParams(i, :);

        fprintf('Parameter combination %d:\n', i)
        disp(params)
        fprintf('\n')

        figure('name', 'Results for parameter combination ' + string(i));

        for j=1:size(outData, 2)
            results = outData{i, j};
            models = results{1};

            subplot(ceil(size(outData, 2)/2), 2, j)
            title('Point Cloud ' + string(j))
            % Plot the point cloud
            pcshow(results{2})
            hold on

            % Plot the cuboids
            for k=1:length(models{3})
                plot(models{3}{k})
            end

            % Plot the ground plane
            plot(models{1})
            hold off

        end

        input('Press any key to continue...')
    end
end

end