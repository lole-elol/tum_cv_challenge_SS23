% Script to test the reconstruct3DMultiview function
% and plot the point cloud generated by it

% dataDir = "test/old_computer";
dataDir = "test/delivery_area_dslr_undistorted";

if exist('images','var') == 0  % Load the images if they are not already loaded yet
    images = util.loadImages(dataDir + "/images", log=true, numImages=10);
end

% Load the camera parameters
cameraParams = logic.reconstruct3D.loadCameraParams(dataDir + "/cameras.txt");

% Reconstruct3d multiview
fprintf('\n Reconstructing 3D...\n')
[pointCloudInstance, camPoses, tracks] = logic.reconstruct3DMultiview(images, cameraParams);

% Plot the point cloud
denoisedPointCloud = pcdenoise(pointCloudInstance);  % TODO: tweak this or simpply do this step in cuboid fitting, only test
plotting.plotPointCloud(denoisedPointCloud, camPoses, cameraSizePlotSize=0.2)