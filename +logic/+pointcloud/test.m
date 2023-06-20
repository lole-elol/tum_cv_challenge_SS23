%P = logic.pointcloud.loadData("test/kicker_dslr_undistorted/points3D_massaged.txt");
P = logic.pointcloud.loadData("test/delivery_area_dslr_undistorted/points3D_massaged.txt");


P = logic.pointcloud.filter(P, 3);

% [ceilingPlane, pcCeiling1, pc] = logic.pointcloud.ceilPlane(P, maxDistance=0.2, percentage=0.3, refVector=[0 0 1]);
% [~, pc, pcFloor, floorPlane] = logic.pointcloud.groundPlane(P);
% [ceilingPlane, pcCeiling2, pc] = logic.pointcloud.ceilPlane(P, maxDistance=0.01, percentage=0.1, refVector=[0 0 1]);

logic.pointcloud.ceilPlane2(P);
% pc = pcdenoise(pc, 'NumNeighbors', 50, 'Threshold', 0.5);
% pcshowpair(pc, pcCeiling2);
% hold on
% plot(ceilingPlane);
% figure
% pcshowpair(pc, pcde);





%%%%%%%%%%%%%%%%%%%%% WALLS

% pointCloud = pcdenoise(pc, 'NumNeighbors', 50, 'Threshold', 0.5);
% maxNumPlanes = 100;
% maxMeanError = 0.01;
% planesPointCloud = cell(1, maxNumPlanes);
% geoPlanesModel = cell(1, maxNumPlanes);
% i = 1;
% meanError = 0;

% while meanError < maxMeanError && i < maxNumPlanes
%     [geoPlaneModel,inlierIndices,outlierIndices, meanError] =  pcfitplane(pointCloud,0.015 );
%     % Remove the plane from the point cloud
%     remainPointCloud = select(pointCloud,outlierIndices);
%     planePointCloud = select(pointCloud,inlierIndices);
%     % Update the point cloud
%     pointCloud = remainPointCloud;
%     % Save the plane and its point cloud
%     planesPointCloud{i} = (planePointCloud);
%     geoPlanesModel{i} = (geoPlaneModel);
%     i = i + 1;
% end

% planes = logic.pointcloud.planeFilter(geoPlanesModel);

% pcshow(pc)
% hold on

% for i = 1:length(planes)
%     plot(planes{i})
% end

%%%%%%%%%%%%%%%%





% [models, pc, pcRemaining] = logic.modelDetection(P)
% pcshow(pc)
% hold on
% for i = 1:length(models{3})
%     plot(models{3}{i})
% end