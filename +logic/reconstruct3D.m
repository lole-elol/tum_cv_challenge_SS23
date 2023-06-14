function [point_cloud, cam_poses, matched_points] = reconstruct3D(image_1, image_2, camera_params, varargin)
    % This function takes a set of images and a set of camera parameters and returns a 3D point cloud of the environment.
    % Ressources:
    %   https://de.mathworks.com/help/vision/ug/structure-from-motion-from-two-views.html
    %   https://de.mathworks.com/help/vision/ug/structure-from-motion-from-multiple-views.html
    %   https://de.mathworks.com/support/search.html/answers/153348-tips-and-tricks-about-3d-scene-reconstruction.html?fq%5B%5D=asset_type_name:answer&fq%5B%5D=category:vision/stereo-vision&page=1
    % Inputs:
    %   image_1, image_2: The two images to reconstruct the 3D environment from
    %   camera_params: The camera parameters of the camera used to take the images. This is a struct of type cameraParameters
    % Inputs (optional):
    %   min_quality_1: The minimum quality of the features to detect in the first detection
    %   min_quality_2: The minimum quality of the features to detect in the second detection (smaller to get more features)
    %   roi_border: The border of the region of interest (ROI) in the images. This is the number of pixels from the border of the image (second detection)
    %   e_max_distance: The maximum distance between the epipolar lines and the corresponding points in the second image
    %   e_confidence: The confidence of the epipolar lines
    %   e_max_num_trials: The maximum number of trials to find the epipolar lines
    %   max_reprojection_error: The maximum reprojection error of the points
    %   max_z: The maximum z value of the points (if points too far away, they are not considered)
    % Output:
    %   point_cloud: A 3D point cloud of the environment
    %   cam_poses: The camera poses of the images
    %   matched_points: The matched points of the images. Mx4 matrix with the x and y coordinates of the points in the first and second image

    %% === 0. Parse input arguments ===
    p = inputParser;
    p.addOptional('min_quality_1', 0.1);
    p.addOptional('min_quality_2', 0.01);
    p.addOptional('e_max_distance', 0.2);
    p.addOptional('e_confidence', 99.99);
    p.addOptional('e_max_num_trials', 10000);
    p.addOptional('roi_border', 200);
    p.addOptional('max_reprojection_error', 100);
    % p.addOptional('max_z', 100);
    p.parse(varargin{:});

    min_quality_1 = p.Results.min_quality_1;
    min_quality_2 = p.Results.min_quality_2;
    roi_border = p.Results.roi_border;
    e_max_distance = p.Results.e_max_distance;
    e_confidence = p.Results.e_confidence;
    e_max_num_trials = p.Results.e_max_num_trials;
    max_reprojection_error = p.Results.max_reprojection_error;
    % max_z = p.Results.max_z;
    % ==================================

    %% === 1. Preprocessing ===
    [image_1_preprocessed, image_1_gray, image_1_canny] = logic.preprocessImage(image_1, camera_params);
    [image_2_preprocessed, image_2_gray, image_2_canny] = logic.preprocessImage(image_2, camera_params);
    image_1_preprocessed  = image_1_gray;
    image_2_preprocessed  = image_2_gray;

    %% ===  2. Feature detection and matching ===
    [matched_points1, matched_points2] = logic.extractCommonFeatures(image_1_preprocessed, image_2_preprocessed, camera_params, min_quality=min_quality_1, roi_border=0);
    
    %% === 3. Epipolar geometry: estimate essential matrix and relative pose of the cameras ===
    [E, rel_pose, status] = logic.getEpipolarGeometry(matched_points1, matched_points2, camera_params, ...
        e_max_distance=e_max_distance, e_confidence=e_confidence, e_max_num_trials=e_max_num_trials);
    if status ~= 0
        error("Could not estimate the essential matrix");
    end

    %% === 4. Triangulation === 
    % Get more features from the images to generate a bigger point cloud
    [matched_points1, matched_points2] = logic.extractCommonFeatures(image_1_preprocessed, image_2_preprocessed, camera_params, min_quality=min_quality_2, roi_border=roi_border);
    % Triangulate the points
    point_cloud = logic.getTriangulatedPoints(matched_points1, matched_points2, camera_params, rel_pose, image_1, max_reprojection_error);

    %% === 5. prepare output ===
    cam_poses = [];
    cam_poses = [cam_poses, rigidtform3d];
    cam_poses = [cam_poses, rel_pose];

    matched_points = [matched_points1.Location, matched_points2.Location];