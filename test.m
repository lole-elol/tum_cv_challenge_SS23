% image1 = imread("kicker/images/dslr_images_undistorted/DSC_6492.JPG");
% image2 = imread("kicker/images/dslr_images_undistorted/DSC_6493.JPG");
% cameraParams = logic.loadCameraParams("kicker/dslr_calibration_undistorted/cameras.txt");

image1 = imread("test/image7.jpg");
image2 = imread("test/image8.jpg");

% show images
% figure(1);
% imshow(image1);
% figure(2);
% imshow(image2);

savePointCloud = false;
loadPointCloud = false;
loadFile = "test/point_clouds/point_cloud06-16-2023_15-00-32.ply";


if loadPointCloud
    pointCloudInstance = pcread(loadFile);
else
    % load camera params from file "test/params/cameraParams.mat"
    cameraParams = load("test/params/camera_params.mat").camera_params;
    tic;
    [pointCloudInstance, relPose, matchedPoints] = logic.reconstruct3D(image1, image2, cameraParams);
    toc;
end

if savePointCloud
    dateNow = datestr(now,'mm-dd-yyyy_HH-MM-SS');
    pcwrite(pointCloudInstance, "test/point_clouds/pointCloud"+ dateNow +".ply");
end


camPoses = [rigidtform3d, relPose];
% show matched points and point cloud
plotting.plotMatchedPoints(image1, image2, matchedPoints);
plotting.plotPointCloud(pointCloudInstance, camPoses);