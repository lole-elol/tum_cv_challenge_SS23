function [groundPtsIdx,remainingPtCloud, groundPlanePtCloud, geoGroundPlane] = groundPlane(ptCloud)
%groundPlane Extracts the ground plane from a point cloud
[groundPtsIdx,remainingPtCloud,groundPointCloud] = segmentGroundSMRF(ptCloud);

[geoGroundPlane, groundPlanePtCloud] = logic.pointcloud.plane(groundPointCloud, maxDistance=0.3)


end