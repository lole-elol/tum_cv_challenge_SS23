%% This file contains all functions necessary to do a 3D reconstruction of the environment
% from a set of 2D images. The main function is "reconstruct3D.m" which takes a set of
% images and a set of camera parameters and returns a 3D point cloud of the environment.

function point_cloud = reconstruct3D(I1, I2, camera_params)
    % This function takes a set of images and a set of camera parameters and returns a 3D point cloud of the environment.
    % Input:
    %   I1, I2: The two images to reconstruct the 3D environment from
    %   camera_params: The camera parameters of the camera used to take the images. This is a struct of type cameraParameters
    % Output:
    %   point_cloud: A 3D point cloud of the environment

    % if set to true, the function will display immediately the results of each step like size of images, detected features, matched features, etc.
    % This is useful for debugging. Each time we will print some description like "Size of x is ..."
    debugging = true;  
    % Convert the images to grayscale
    I1_gray = rgb2gray(I1);
    I2_gray = rgb2gray(I2);

    % Remove lens distortion from the images
    I1_gray = undistortImage(I1_gray, camera_params);
    I2_gray = undistortImage(I2_gray, camera_params);

    % Detect features in the images TODO consider changing detection algorithm
    points1 = detectMinEigenFeatures(I1_gray, MinQuality = 0.1);
    points2 = detectMinEigenFeatures(I2_gray, MinQuality = 0.1);

    if debugging
        disp("Size of points1 is ");
        disp(size(points1));
        disp("Size of points2 is ");
        disp(size(points2));
    end

    % Extract features from the images
    [features1, valid_points1] = extractFeatures(I1_gray, points1);
    [features2, valid_points2] = extractFeatures(I2_gray, points2);
    if debugging
        disp("Size of features1 is ");
        disp(size(features1));
        disp("Size of features2 is ");
        disp(size(features2));
        disp("Size of valid_points1 is ");
        disp(size(valid_points1));
        disp("Size of valid_points2 is ");
        disp(size(valid_points2));
    end
    % UNCOMMENT IF DEBUGGING
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


    % Match the features between the images
    index_pairs = matchFeatures(features1, features2);
    if debugging
        disp("Size of index_pairs (number of paired features) is ");
        disp(size(index_pairs));
    end

    % Retrieve the locations of the corresponding points for each image
    matched_points1 = valid_points1(index_pairs(:, 1));
    matched_points2 = valid_points2(index_pairs(:, 2));

    % Show matched points in an image
    % UNCOMMENT IF DEBUGGING
    % figure;
    % showMatchedFeatures(I1_gray, I2_gray, matched_points1, matched_points2);
    % legend("matched points 1","matched points 2");

    % Estimate the essential matrix
    [E, inliers, status] = estimateEssentialMatrix(matched_points1, matched_points2, camera_params, 'Confidence', 99.99, 'MaxNumTrials', 10000, 'MaxDistance', 0.2);
    if status ~= 0
        error("Could not estimate the essential matrix");
    end
    inlier_points1 = matched_points1(inliers);
    inlier_points2 = matched_points2(inliers);
    % Show matches after applying the RANSAC algorithm
    % UNCOMMENT IF DEBUGGING
    % figure;
    % showMatchedFeatures(I1_gray, I2_gray, matched_points1(inliers), matched_points2(inliers));
    % legend("matched points 1","matched points 2");

    % Compute the camera pose from the essential matrix using estrelpose
    relPose = estrelpose(E, camera_params.Intrinsics, camera_params.Intrinsics, inlier_points1, inlier_points2);
    if debugging
        disp("Relative pose is ");
        disp(relPose);
    end

    % Recompute matched points but only using points that are in the middle of the image
    border = 150;  % TODO consider changing this value, dont hard code it
    roi = [border, border, size(I1_gray, 2) - 2 * border, size(I1_gray, 1) - 2 * border];
    points1 = detectMinEigenFeatures(I1_gray, 'ROI', roi, MinQuality = 0.2);
    roi = [border, border, size(I2_gray, 2) - 2 * border, size(I2_gray, 1) - 2 * border];   
    points2 = detectMinEigenFeatures(I2_gray, 'ROI', roi, MinQuality = 0.2);
    [features1, valid_points1] = extractFeatures(I1_gray, points1);
    [features2, valid_points2] = extractFeatures(I2_gray, points2);
    index_pairs = matchFeatures(features1, features2);
    matched_points1 = valid_points1(index_pairs(:, 1));
    matched_points2 = valid_points2(index_pairs(:, 2));
    % UNCOMMENT IF DEBUGGING
    figure;
    showMatchedFeatures(I1, I2, matched_points1, matched_points2);
    legend("matched points 1","matched points 2");

    % Compute the 3D points from the camera pose
    camMatrix1 = cameraProjection(camera_params.Intrinsics, rigidtform3d);
    camMatrix2 = cameraProjection(camera_params.Intrinsics, pose2extr(relPose));
    [point_cloud, reprojection_error, valid_index] = triangulate(matched_points1, matched_points2, camMatrix1, camMatrix2);
    point_cloud = point_cloud ./ 100;  % convert from cm to m TODO check this value
    
    % Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
    % remove points with high reprojection error
    max_reprojection_error = 100;
    max_z = 5;
    point_cloud = point_cloud(valid_index & point_cloud(:, 3) < max_z & reprojection_error < max_reprojection_error, :);

    % UNCOMMENT IF DEBUGGING
    figure;
    hold on;
    % Show cameras pose
    plotCamera('Size', 0.05, 'Color', 'r', 'Label', '1');  % plot first camera
    plotCamera('Size', 0.05, 'Color', 'b', 'Label', '2', 'AbsolutePose', relPose);  % plot second camera
    % Show point cloud and increase point size
    pcshow(point_cloud,'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 45);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');

    % Rotate and zoom the plot
    camorbit(0, -30);

    hold off;

    % Get the number of images
    % num_images = length(images);
    % features = cell(num_images, 1);
    % valid_points = cell(num_images, 1);

    % % Make images gray
    % for i = 1:num_images
    %     images{i} = rgb2gray(images{i});
    % end

    % % Apply a detection algorithm to each image to extract the features
    % for i = 1:num_images
    %     % Detect features in the image
    %     points = detectHarrisFeatures(images{i});
    %     [features{i}, valid_points{i}] = extractFeatures(images{i}, points);
    % end

    % % Match the features between the images
    % index_pairs = cell(num_images - 1, 1);
    % matched_points1 = cell(num_images - 1, 1);
    % matched_points2 = cell(num_images - 1, 1);

    % for i = 1:num_images - 1
    %     % Match the features between the images
    %     index_pairs{i} = matchFeatures(features{i}, features{i + 1});
    %     matched_points1{i} = valid_points{i}(index_pairs{i}(:, 1));
    %     matched_points2{i} = valid_points{i + 1}(index_pairs{i}(:, 2));
    % end

    % % Show matched points in an image for the 2 first images
    % figure;
    % showMatchedFeatures(images{1}, images{2}, matched_points1{1}, matched_points2{1});
    % legend("matched points 1","matched points 2");

