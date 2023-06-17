function [geoPlanesModel, planesPointCloud] = plane(pointCloud, varargin)

% PLANE -   This function returns the planes of a point cloud
%   [geoPlanesModel, planesPointCloud] = plane(pointCloud, maxDistance, varargin)
% Inputs:
%   pointCloud: Point cloud to be segmented
%   maxDistance = 0.015: Maximum distance from an inlier point to the plane.
%   refVector = [0 0 1]: Reference vector for plane normal.
%   maxAngularDistance = 1: Maximum angle in degrees between the normal
%   maxNumPlanes = 10: Maximum number of planes to be segmented.
%   maxMeanError = 0.01: Maximum mean error of the planes.
% Outputs:
%   geoPlanesModel: Geometric plane model.
%   planesPointCloud: Point cloud of the planes.

p = inputParser;
addRequired(p,'pointCloud',@(x) isa(x,'pointCloud'));
addOptional(p,'maxDistance', 0.015,@isnumeric);
addOptional(p,'refVector',[0 0 1]);
addOptional(p,'maxAngularDistance',1,@isnumeric);
addOptional(p, 'maxNumPlanes', 10, @isnumeric);
addOptional(p, 'maxMeanError', 0.01, @isnumeric);

parse(p,pointCloud,varargin{:});
maxDistance = p.Results.maxDistance;
refVector = p.Results.refVector;
maxAngularDistance = p.Results.maxAngularDistance;
maxNumPlanes = p.Results.maxNumPlanes;
maxMeanError = p.Results.maxMeanError;

% Initialize variables
planesPointCloud = cell(1, maxNumPlanes);
geoPlanesModel = cell(1, maxNumPlanes);
i = 1;
meanError = 0;

% Segment planes
while meanError < maxMeanError && i < maxNumPlanes
    [geoPlaneModel,inlierIndices,outlierIndices, meanError] =  pcfitplane(pointCloud,maxDistance, refVector, maxAngularDistance);
    % Remove the plane from the point cloud
    remainPointCloud = select(pointCloud,outlierIndices);
    planePointCloud = select(pointCloud,inlierIndices);
    % Update the point cloud
    pointCloud = remainPointCloud;
    % Save the plane and its point cloud
    planesPointCloud{i} = (planePointCloud);
    geoPlanesModel{i} = (geoPlaneModel);
    i = i + 1;
end

end

