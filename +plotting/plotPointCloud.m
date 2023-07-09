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
p.addOptional('cameraSizePlotSize', 0.2, @isnumeric);
p.addOptional('pcMarkerSize', 45, @isnumeric);
p.addOptional('pcViewerMarkerSize', 5, @isnumeric);
p.parse(varargin{:});
usePcViewer = p.Results.usePcViewer;
cameraSizePlotSize = p.Results.cameraSizePlotSize;
pcMarkerSize = p.Results.pcMarkerSize;
pcViewerMarkerSize = p.Results.pcViewerMarkerSize;


if usePcViewer
    pcviewer(pointCloudInstance, PointSize=pcViewerMarkerSize);
else
    hold on;
    pcshow(pointCloudInstance, VerticalAxisDir='Down', MarkerSize=pcMarkerSize);
    % Show cameras poses
    if ~isempty(camPoses)
        plotCamera(camPoses, Size=cameraSizePlotSize);
    end
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end
camorbit(180,0, "data", [0, 1, 0])
camorbit(180,0, "data", [0, 0, 1]) 