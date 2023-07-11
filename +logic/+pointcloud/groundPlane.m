function [groundPtsIdx, remainingPtCloud, groundPlanePtCloud, geoGroundPlane] = groundPlane(ptCloud, varargin)
% GROUNDPLANE Find the ground plane in a point cloud
%Input:
%   ptCloud: point cloud
%   maxDistance = 0.3: maximum distance from the plane for a point to be
%Output:
%   groundPtsIdx: indices of the points in the ground plane
%   remainingPtCloud: point cloud without the ground plane
%   groundPlanePtCloud: point cloud of the ground plane
%   geoGroundPlane: geometric plane of the ground plane

p = inputParser;
p.addOptional('maxDistance', 0.3);

p.parse(varargin{:});
maxDistance = p.Results.maxDistance;

[groundPtsIdx,remainingPtCloud,groundPointCloud] = segmentGroundSMRF(ptCloud, MaxWindowRadius=10, SlopeThreshold=1, ElevationThreshold=1, ElevationScale=2);

[geoGroundPlanes, groundPlanesPtCloud] = logic.pointcloud.plane(groundPointCloud, maxDistance=maxDistance);

geoGroundPlane = geoGroundPlanes{1};
groundPlanePtCloud = groundPlanesPtCloud{1};

end