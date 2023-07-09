function [groundPtsIdx, remainingPtCloud, groundPlanePtCloud, geoGroundPlane] = groundPlane(ptCloud)
% GROUNDPLANE Find the ground plane in a point cloud
%Input:
%   ptCloud: point cloud
%Output:
%   groundPtsIdx: indices of the points in the ground plane
%   remainingPtCloud: point cloud without the ground plane
%   groundPlanePtCloud: point cloud of the ground plane
%   geoGroundPlane: geometric plane of the ground plane


[groundPtsIdx,remainingPtCloud,groundPointCloud] = segmentGroundSMRF(ptCloud);

[geoGroundPlanes, groundPlanesPtCloud] = logic.pointcloud.plane(groundPointCloud, maxDistance=0.3);

geoGroundPlane = geoGroundPlanes{1};
groundPlanePtCloud = groundPlanesPtCloud{1};

end