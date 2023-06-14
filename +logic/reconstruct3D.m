function [point_cloud, cam_poses] = reconstruct3D(I1, I2, camera_params, varargin)
    % This function takes a set of images and a set of camera parameters and returns a 3D point cloud of the environment.
    % Ressources:
    %   https://de.mathworks.com/help/vision/ug/structure-from-motion-from-two-views.html
    %   https://de.mathworks.com/help/vision/ug/structure-from-motion-from-multiple-views.html
    %   https://de.mathworks.com/support/search.html/answers/153348-tips-and-tricks-about-3d-scene-reconstruction.html?fq%5B%5D=asset_type_name:answer&fq%5B%5D=category:vision/stereo-vision&page=1
    % Inputs:
    %   I1, I2: The two images to reconstruct the 3D environment from
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

    % if set to true, the function will display immediately the results of each step like size of images, detected features, matched features, etc.
    % This is useful for debugging. Each time we will print some description like "Size of x is ..."
    debugging = false;
    %% 0. Parse input arguments with parseInputArguments()
    p = inputParser;
    p.addOptional('min_quality_1', 0.1);
    p.addOptional('min_quality_2', 0.05);
    p.addOptional('e_max_distance', 0.2);
    p.addOptional('e_confidence', 99.99);
    p.addOptional('e_max_num_trials', 10000);
    p.addOptional('roi_border', 200);
    p.addOptional('max_reprojection_error', 100);
    p.addOptional('max_z', 100);
    p.parse(varargin{:});

    min_quality_1 = p.Results.min_quality_1;
    min_quality_2 = p.Results.min_quality_2;
    roi_border = p.Results.roi_border;
    e_max_distance = p.Results.e_max_distance;
    e_confidence = p.Results.e_confidence;
    e_max_num_trials = p.Results.e_max_num_trials;
    max_reprojection_error = p.Results.max_reprojection_error;
    max_z = p.Results.max_z;

    %% 1. Preprocessing
    % Convert the images to grayscale TODO: consider using color informaiton for the reconstruction
    I1_gray = rgb2gray(I1);
    I2_gray = rgb2gray(I2);

    % Remove lens distortion from the images
    % I1_gray = undistortImage(I1_gray, camera_params);
    % I2_gray = undistortImage(I2_gray, camera_params);

    % TODO: Apply a canny detector to the images to get the edges of the objects in the images
    % I1_gray_canny = edge(I1_gray, 'Canny');
    % I2_gray_canny = edge(I2_gray, 'Canny');

    % TODO: Take a look at Hough transformation (i.e Fourier transform) to get lines in the images

    % Detect features in the images 
    % TODO: consider changing detection algorithm and getting different point types
    % https://de.mathworks.com/help/vision/ug/point-feature-types.html
    
    %% 2. Feature detection and matching
    points1 = detectMinEigenFeatures(I1_gray, MinQuality = min_quality_1);
    points2 = detectMinEigenFeatures(I2_gray, MinQuality = min_quality_1);

    if debugging
        disp("Points1 is ");
        disp(points1);
        disp("Points2 is ");
        disp(points2);
    end

    % Extract features from the images
    [features1, valid_points1] = extractFeatures(I1_gray, points1);
    [features2, valid_points2] = extractFeatures(I2_gray, points2);
    if debugging
        disp("features1 is ");
        disp(features1);
        disp("features2 is ");
        disp(features2);
        disp("valid_points1 is ");
        disp(valid_points1);
        disp("valid_points2 is ");
        disp(valid_points2);
    end

    % === UNCOMMENT IF DEBUGGING ===
    % figure;
    % imshow(I1_gray);
    % hold on;
    % plot(valid_points1.selectStrongest(20));
    % hold off;
    % figure;
    % imshow(I2_gray);
    % hold on;
    % plot(valid_points2.selectStrongest(20));
    % hold off;
    % ==============================


    % Match the features between the images
    index_pairs = matchFeatures(features1, features2);
    if debugging
        disp("index_pairs size is ");
        disp(size(index_pairs));
    end

    % Retrieve the locations of the corresponding points for each image
    matched_points1 = valid_points1(index_pairs(:, 1));
    matched_points2 = valid_points2(index_pairs(:, 2));

    if debugging
        disp("matched_points1 is ");
        disp(matched_points1);
        disp("matched_points2 is ");
        disp(matched_points2);
    end

    % === UNCOMMENT IF DEBUGGING ===
    % Show matched points in an image
    % figure;
    % showMatchedFeatures(I1_gray, I2_gray, matched_points1, matched_points2);
    % legend("matched points 1","matched points 2");
    % ==============================

    % Estimate the essential matrix
    [E, inliers, status] = estimateEssentialMatrix(matched_points1, matched_points2, camera_params, ...
        'MaxDistance', e_max_distance, 'Confidence', e_confidence, 'MaxNumTrials', e_max_num_trials);
    if status ~= 0
        error("Could not estimate the essential matrix");
    end
    inlier_points1 = matched_points1(inliers);
    inlier_points2 = matched_points2(inliers);

    % === UNCOMMENT IF DEBUGGING ===
    % Show matches after applying the RANSAC algorithm
    % figure;
    % showMatchedFeatures(I1_gray, I2_gray, matched_points1(inliers), matched_points2(inliers));
    % legend("matched points 1","matched points 2");
    % ==============================

    %% 3. Relative pose estimation
    rel_pose = estrelpose(E, camera_params.Intrinsics, camera_params.Intrinsics, inlier_points1, inlier_points2);
    if debugging
        disp("Relative pose is ");
        disp(relPose);
    end

    % Recompute matched points but only using points that are in the middle of the image
    % TODO: try to get a lot of points so that we can use more points for the triangulation and get a larger point cloud
    roi = [roi_border, roi_border, size(I1_gray, 2) - 2 * roi_border, size(I1_gray, 1) - 2 * roi_border];
    points1 = detectMinEigenFeatures(I1_gray, 'ROI', roi, MinQuality = min_quality_2);
    roi = [roi_border, roi_border, size(I2_gray, 2) - 2 * roi_border, size(I2_gray, 1) - 2 * roi_border];   
    points2 = detectMinEigenFeatures(I2_gray, 'ROI', roi, MinQuality = min_quality_2);

    [features1, valid_points1] = extractFeatures(I1_gray, points1);
    [features2, valid_points2] = extractFeatures(I2_gray, points2);
    index_pairs = matchFeatures(features1, features2);
    matched_points1 = valid_points1.Location(index_pairs(:, 1), :);
    matched_points2 = valid_points2.Location(index_pairs(:, 2), :);

    % === UNCOMMENT IF DEBUGGING ===
    figure;
    showMatchedFeatures(I1, I2, matched_points1, matched_points2);
    legend("matched points 1","matched points 2");
    % ==============================

    % Compute the 3D points from the camera pose
    camMatrix1 = cameraProjection(camera_params.Intrinsics, rigidtform3d);
    camMatrix2 = cameraProjection(camera_params.Intrinsics, pose2extr(rel_pose));
    [world_points, reprojection_error, valid_index] = triangulate(matched_points1, matched_points2, camMatrix1, camMatrix2);
    % TODO: do bundle adjustment

    % Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
    % remove points with high reprojection error
    valid_index = valid_index & (world_points(:, 3) > 0) & (reprojection_error < max_reprojection_error);
    world_points = world_points(valid_index, :);
    

    % Get the color of each reconstructed point
    numPixels = size(I1, 1) * size(I1, 2);
    allColors = reshape(I1, [numPixels, 3]);
    colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matched_points1(valid_index, 2)), round(matched_points1(valid_index, 1)));
    
    color = allColors(colorIdx, :);
    point_cloud = pointCloud(world_points, 'Color', color);

    cam_poses = [];
    cam_poses = [cam_poses, rigidtform3d];
    cam_poses = [cam_poses, rel_pose];