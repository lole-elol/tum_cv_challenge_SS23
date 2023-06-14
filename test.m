% I1 = imread("kicker/images/dslr_images_undistorted/DSC_6492.JPG");
% I2 = imread("kicker/images/dslr_images_undistorted/DSC_6493.JPG");
% camera_params = logic.loadCameraParams("kicker/dslr_calibration_undistorted/cameras.txt");

I1 = imread("test/image7.jpg");
I2 = imread("test/image8.jpg");

% show images
% figure(1);
% imshow(I1);
% figure(2);
% imshow(I2);

% load camera params from file "test/params/camera_params.mat"
camera_params = load("test/params/camera_params.mat").camera_params;

[pc, cam_poses] = logic.reconstruct3D(I1, I2, camera_params);
disp(cam_poses)
% show point cloud
plotting.plotPointCloud(pc, cam_poses);