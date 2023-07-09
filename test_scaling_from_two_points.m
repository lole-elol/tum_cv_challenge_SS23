%% Test scaling from two points (delivery area, first 2 images, door)
% We assume that reconstruction is done and that the point cloud is
% available as well as the camera poses in the workspace
% Run test_multiview first.

% Close open figures
close all

% Image 1 Points
p11 = [354, 3955];
p12 = [719, 875];
% Image 2 Points
p21 = [1144, 3903];
p22 = [1390, 870];
% Estimated door height
height = 2.1;  % meters
points = {[p11; p21], [p12; p22]};
% points_ = {[p11; p12], [p21; p22]};
% showMatchedFeatures(images{1}, images{2}, points_{1}, points_{2});

% Camera poses
camPose1 = camPoses(1, :).AbsolutePose;
camPose2 = camPoses(2, :).AbsolutePose;
[scalingFactor, markedPoints] = logic.pointcloud.scalingFactorFrom2Points(points, camPoses(1:2, :), cameraParams, height);

% Scale the scene using a scaling transformation
tScaling = affinetform3d([scalingFactor, 0, 0, 0; 0, scalingFactor, 0, 0; 0, 0, scalingFactor, 0; 0, 0, 0, 1]);
[denoisedPointCloudScaled, camPosesScaled] = logic.reconstruct3D.transformScene(denoisedPointCloud, camPoses, tScaling);

% Plot the point cloud unscaled as well as the points of the door frame
figure
plotting.plotPointCloud(denoisedPointCloud, camPoses, cameraSizePlotSize=0.2, pcMarkerSize=10)
plotting.plotPointCloud(pointCloud(markedPoints, Color=[0,0,1]), [], pcMarkerSize=500);
xlabel('X [Unknown]');
ylabel('Y [Unknown]');
zlabel('Z [Unknown]');
hold off

% Plot the point cloud scaled
figure
plotting.plotPointCloud(denoisedPointCloudScaled, camPosesScaled, cameraSizePlotSize=0.2*scalingFactor, pcMarkerSize=10)
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
hold off