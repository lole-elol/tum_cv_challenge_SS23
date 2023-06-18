function [groundPtsIdx,remainingPtCloud, groundPlanePtCloud, geoGroundPlane] = groundPlane(ptCloud)
%groundPlane Extracts the ground plane from a point cloud
[groundPtsIdx,remainingPtCloud,groundPointCloud] = segmentGroundSMRF(ptCloud);

[geoGroundPlanes, groundPlanesPtCloud] = logic.pointcloud.plane(groundPointCloud, maxDistance=0.3);

geoGroundPlane = geoGroundPlanes{1};
groundPlanePtCloud = groundPlanesPtCloud{1};

end