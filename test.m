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
tic;
[pc, rel_pose, matched_points] = logic.reconstruct3D(I1, I2, camera_params);
toc;
cam_poses = [rigidtform3d, rel_pose];
% show matched points and point cloud
plotting.plotMatchedPoints(I1, I2, matched_points);
plotting.plotPointCloud(pc, cam_poses);