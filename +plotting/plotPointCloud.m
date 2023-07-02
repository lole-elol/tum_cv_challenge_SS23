function plotPointCloud(pointCloudInstance, camPoses, varargin)
    % PLOTPOINTCLOUD Plot the point cloud with the camera poses
    % Inputs:
    %   pointCloudInstance: 3D point cloud as a pointCloud object
    %   camPoses: camera poses as an array of rigid3d objects
    %   usePcViewer = false: use the pcviewer function instead of pcshow
    %   cameraSizePlotSize = 1.0: size of the camera plot
    %   pcMarkerSize = 45: size of the point cloud markers
    % Outputs:
    %   None

    % Define parser
    p = inputParser;
    p.addOptional('usePcViewer', false, @islogical);
    p.addOptional('cameraSizePlotSize', 1.0, @isnumeric);
    p.addOptional('pcMarkerSize', 45, @isnumeric);
    p.parse(varargin{:});
    usePcViewer = p.Results.usePcViewer;
    cameraSizePlotSize = p.Results.cameraSizePlotSize;
    pcMarkerSize = p.Results.pcMarkerSize;

    hold on;

    if usePcViewer
        pcviewer(pointCloudInstance, PointSize=1);
    else
        pcshow(pointCloudInstance, VerticalAxisDir='Down', MarkerSize=pcMarkerSize);
        % Show cameras poses
        plotCamera(camPoses);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
    end
    % Rotate and zoom the plot
    camorbit(0, -30);