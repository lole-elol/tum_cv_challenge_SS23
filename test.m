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

save_pc = false;
load_pc = false;
load_file = "test/point_clouds/point_cloud06-16-2023_15-00-32.ply";


if load_pc
    point_cloud = pcread(load_file);
else
    % load camera params from file "test/params/camera_params.mat"
    camera_params = load("test/params/camera_params.mat").camera_params;
    tic;
    [point_cloud, rel_pose, matched_points] = logic.reconstruct3D(I1, I2, camera_params);
    toc;
end

if save_pc
    date_now = datestr(now,'mm-dd-yyyy_HH-MM-SS');
    pcwrite(point_cloud, "test/point_clouds/point_cloud"+ date_now +".ply");
end


cam_poses = [rigidtform3d, rel_pose];
% show matched points and point cloud
plotting.plotMatchedPoints(I1, I2, matched_points);
plotting.plotPointCloud(point_cloud, cam_poses);