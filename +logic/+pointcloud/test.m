%%%%%%%%%%%%%%%%%% EVALUATE


% P1 = logic.pointcloud.loadData("test\delivery_area_dslr_undistorted\points3D_massaged.txt");
% P2 = logic.pointcloud.loadData("test\kicker_dslr_undistorted\points3D_massaged.txt");
% P3 = logic.pointcloud.loadData("test\pipes\points3D_massaged.txt");
% P4 = logic.pointcloud.loadData("test\relief\points3D_massaged.txt");
% P5 = logic.pointcloud.loadData("test\terrains\points3D_massaged.txt");
% P6 = logic.pointcloud.loadData("test\3D_reconstruct\points3D_reconstruct3D_kicker.txt");

P7 = logic.pointcloud.loadData("test\3D_reconstruct\points3D_reconstruct_deliveryArea1.txt");
% P8 = logic.pointcloud.loadData("test\3D_reconstruct\points3D_reconstruct_deliveryArea2.txt");
% P9 = logic.pointcloud.loadData("test\3D_reconstruct\points3D_reconstruct_deliveryArea3.txt");

%%%%  Playing with rotation %%%
rotationAngles = [90 0 0];
translation = [0 0 0];

tform = rigidtform3d(rotationAngles,translation);
P7 = pctransform(P7,tform);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% [models1, pc1, pcRemaining1] = logic.modelDetection(P1 );
% [models2, pc2, pcRemaining2] = logic.modelDetection(P2 );
% [models3, pc3, pcRemaining3] = logic.modelDetection(P3 );
% [models4, pc4, pcRemaining4] = logic.modelDetection(P4, ceilingPercentile=0.3);
% [models5, pc5, pcRemaining5] = logic.modelDetection(P5 );
% [models6, pc6, pcRemaining6] = logic.modelDetection(P6, ceilingPercentile=0.3);

[models7, pc7, pcRemaining7] = logic.modelDetection(pcrot);
% [models8, pc8, pcRemaining8] = logic.modelDetection(P8, ceilingPercentile=0.3);
% [models9, pc9, pcRemaining9] = logic.modelDetection(P9, ceilingPercentile=0.3);

% figure; pcshow(pc1);
% hold on
% for i=1:length(models1{3})
%     plot(models1{3}{i});
% end
% plot(models1{1});

% figure; pcshow(pc2);
% hold on
% for i=1:length(models2{3})
%     plot(models2{3}{i});
% end
% plot(models2{1});

% figure; pcshow(pc3);
% hold on
% for i=1:length(models3{3})
%     plot(models3{3}{i});
% end
% plot(models3{1});

% figure; pcshow(pc4);
% hold on
% for i=1:length(models4{3})
%     plot(models4{3}{i});
% end
% plot(models4{1});

% figure; pcshow(pc5);
% hold on
% for i=1:length(models5{3})
%     plot(models5{3}{i});
% end
% plot(models5{1});

% figure; pcshow(pc6);
% hold on
% for i=1:length(models6{3})
%     plot(models6{3}{i});
% end
% plot(models6{1});

figure; pcshow(pc7);
hold on
for i=1:length(models7{3})
    plot(models7{3}{i});
end
plot(models7{1});

% figure; pcshow(pc8);
% hold on
% for i=1:length(models8{3})
%     plot(models8{3}{i});
% end
% plot(models8{1});


% figure; pcshow(pc9);
% hold on
% for i=1:length(models9{3})
%     plot(models9{3}{i});
% end
% plot(models9{1});

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