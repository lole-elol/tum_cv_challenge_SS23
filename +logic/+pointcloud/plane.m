function [geoPlaneModel, planePointCloud, remainPointCloud, meanError] = plane(pointCloud, maxDistance)
% PLANE - fit a plane to a point cloud
% RETURNS:
% A geometrical model of the plane
% The point cloud of the plane iteslef
% The point cloud without the plane points
% The mean error of the plane fitting

[geoPlaneModel,inlierIndices,outlierIndices, meanError] =  pcfitplane(pointCloud,maxDistance);
planePointCloud = select(pointCloud,inlierIndices);
remainPointCloud = select(pointCloud,outlierIndices);




end

% A = cell(1, 10);
% P = cell(1, 10);
% i = 1;
% meanError = 0;
% ptcloud = B
% while meanError < 0.01 && i < 100
%     [geoPlaneModel, planePointCloud, remainPointCloud, meanError] = logic.pointcloud.plane(ptcloud, 0.015);
%     ptcloud = remainPointCloud;
%     A{i} = (planePointCloud);
%     P{i} = (geoPlaneModel);
%     i = i + 1;
% end