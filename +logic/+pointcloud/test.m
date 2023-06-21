%%%%%%%%%%%%%%%%%% EVALUATE


P1 = logic.pointcloud.loadData("test\delivery_area_dslr_undistorted\points3D_massaged.txt");
P2 = logic.pointcloud.loadData("test\kicker_dslr_undistorted\points3D_massaged.txt");
P3 = logic.pointcloud.loadData("test\pipes\points3D_massaged.txt");
% P4 = logic.pointcloud.loadData("test\relief\points3D_massaged.txt");
P5 = logic.pointcloud.loadData("test\terrains\points3D_massaged.txt");


[models1, pc1, pcRemaining1] = logic.modelDetection(P1 );
[models2, pc2, pcRemaining2] = logic.modelDetection(P2 );
[models3, pc3, pcRemaining3] = logic.modelDetection(P3 );
% [models4, pc4, pcRemaining4] = logic.modelDetection(P4 );
[models5, pc5, pcRemaining5] = logic.modelDetection(P5 );

figure; pcshow(pc1);
hold on
for i=1:length(models1{3})
    plot(models1{3}{i});
end
plot(models1{1});


figure; pcshow(pc2);
hold on
for i=1:length(models2{3})
    plot(models2{3}{i});
end
plot(models2{1});

figure; pcshow(pc3);
hold on
for i=1:length(models3{3})
    plot(models3{3}{i});
end
plot(models3{1});

% figure; pcshow(pc4);
% hold on
% for i=1:length(models4{3})
%     plot(models4{3}{i});
% end
% plot(models4{1});

figure; pcshow(pc5);
hold on
for i=1:length(models5{3})
    plot(models5{3}{i});
end
plot(models5{1});

%%%%%%%%%%%%%%%%%%  END EVALUATE







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