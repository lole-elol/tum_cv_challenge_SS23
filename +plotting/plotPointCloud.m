function plotPointCloud(pointCloudInstance, camPoses, varargin)
    % PLOTPOINTCLOUD Plot the point cloud with the camera poses
    % Inputs:
    %   pointCloudInstance: 3D point cloud as a pointCloud object
    %   camPoses: camera poses as an array of rigid3d objects
    %   cameraSizePlotSize = 1.0: size of the camera plot
    %   pcMarkerSize = 45: size of the point cloud markers
    % Outputs:
    %   None

    % Define parser
    p = inputParser;
    p.addOptional('usePcViewer', false);
    p.addOptional('cameraSizePlotSize', 1.0);
    p.addOptional('pcMarkerSize', 45);
    p.parse(varargin{:});
    usePcViewer = p.Results.usePcViewer;
    cameraSizePlotSize = p.Results.cameraSizePlotSize;
    pcMarkerSize = p.Results.pcMarkerSize;

    figure;
    hold on;

    if usePcViewer
        pcviewer(pointCloudInstance,VerticalAxis='YDown', PointSize=1);
    else
        pcshow(pointCloudInstance, VerticalAxis='Y', VerticalAxisDir='Down', MarkerSize=pcMarkerSize);
        % Show cameras poses
        for pose=1:height(camPoses)
            % Plot camera
            if size(camPoses,2) > 1
                camPose = camPoses(pose, 2).AbsolutePose;
            else
                camPose = camPoses(pose);
            end
            plotCamera(Location=camPose.Translation, Orientation=camPose.R, ...
                Size=cameraSizePlotSize, Color='b', Opacity=0, label=string(pose));
        end
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
    end
    % Rotate and zoom the plot
    camorbit(0, -30);
    hold off;