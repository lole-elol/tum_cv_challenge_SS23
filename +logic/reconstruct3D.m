function [point_cloud, rel_pose, matched_points] = reconstruct3D(image_1, image_2, camera_params, varargin)
    % RECONSTRUCT3D Take a set of images and a set of camera parameters and returns a 3D point cloud of the environment.
    % Ressources:
    %   https://de.mathworks.com/help/vision/ug/structure-from-motion-from-two-views.html
    %   https://de.mathworks.com/help/vision/ug/structure-from-motion-from-multiple-views.html
    %   https://de.mathworks.com/support/search.html/answers/153348-tips-and-tricks-about-3d-scene-reconstruction.html?fq%5B%5D=asset_type_name:answer&fq%5B%5D=category:vision/stereo-vision&page=1
    % Inputs:
    %   image_1, image_2: The two images to reconstruct the 3D environment from
    %   camera_params: The camera parameters of the camera used to take the images. This is a struct of type cameraParameters
    %   min_quality_1 = 0.1: The minimum quality of the features to detect in the first detection
    %   min_quality_2 = 0.01: The minimum quality of the features to detect in the second detection
    %   roi_border = 200: The border around the image to ignore when detecting features
    %   e_max_distance = 0.2: The maximum distance of a point to the epipolar line to be considered an inlier
    %   e_confidence = 99.99: The confidence of the estimated essential matrix
    %   e_max_num_trials = 10000: The maximum number of trials to estimate the essential matrix
    %   max_reprojection_error = 100: The maximum reprojection error of a point to be considered an inlier
    % Outputs:
    %   point_cloud: The 3D point cloud of the environment
    %   rel_pose: The relative pose of the second camera to the first camera
    %   matched_points: The matched points between the two images. Mx4 matrix with the x and y coordinates of the matched points in the first and second image

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
    matched_points = [matched_points1.Location, matched_points2.Location];