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
    disp("! NOT IMPLEMENTED YET !")
end

if showDetection
    disp('Displaying 3D Model Detection Results!')
    disp("Loading Benchmark Results...")
    load(fullfile(dataPath, 'detectionParams.mat'), 'detectionParams');
    load(fullfile(dataPath, 'detectionOut.mat'), 'outData');
    fprintf('\n')

    for i=1:size(detectionParams, 1)
        params = detectionParams(i, :);

        disp('Results for parameter combination ' + string(i));
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

        figure('name', 'Results for parameter combination ' + string(i));

        for j=1:size(outData, 2)
            results = outData{i, j};
            models = results{1};

            subplot(ceil(size(outData, 2)/2), 2, j)
            title('Demo ' + string(j))
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