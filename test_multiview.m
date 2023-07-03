% Script to test the reconstruct3DMultiview function
% and plot the point cloud generated by it

% dataDir = "test/old_computer";
dataDir = "test/delivery_area_dslr_undistorted";

if exist('images','var') == 0  % Load the images if they are not already loaded yet
    images = util.loadImages(dataDir + "/images", log=true, numImages=3);
end

% Load the camera parameters
cameraParams = logic.reconstruct3D.loadCameraParams(dataDir + "/cameras.txt");

% Reconstruct3d multiview
fprintf('\n Reconstructing 3D...\n')
[pointCloudInstance, camPoses, tracks] = logic.reconstruct3DMultiview(images, cameraParams);


% Plot the point cloud
denoisedPointCloud = pcdenoise(pointCloudInstance);  % TODO: tweak this or simpply do this step in cuboid fitting, only test


% Test scaling from two points (delivery area, first 2 images, door)
% Image 1
p11 = [354, 3955];
p12 = [719, 875];
% Image 2
p21 = [1144, 3903];
p22 = [1390, 870];
% Estimated door height
height = 2.1;  % meters
points = {[p11; p12], [p21; p22]};
% Camera poses
camPose1 = camPoses(1, :).AbsolutePose;
camPose2 = camPoses(2, :).AbsolutePose;
[scalingFactor, markedPoints] = logic.pointcloud.scalingFactorFrom2Points(points, height, camPose1, camPose2, cameraParams.Intrinsics)
% Scale the point cloud
tScaling = affinetform3d([scalingFactor, 0, 0, 0; 0, scalingFactor, 0, 0; 0, 0, scalingFactor, 0; 0, 0, 0, 1]);
denoisedPointCloudScaled = pctransform(denoisedPointCloud, tScaling);    

plotting.plotPointCloud(denoisedPointCloud, camPoses, cameraSizePlotSize=0.2)
% Plot marked points as well
markedPointsPointCloud = pointCloud(markedPoints, Color=[1,0,0]);
plotting.plotPointCloud(markedPointsPointCloud, []);
hold off
figure
plotting.plotPointCloud(denoisedPointCloudScaled, camPoses, cameraSizePlotSize=0.2)
hold off