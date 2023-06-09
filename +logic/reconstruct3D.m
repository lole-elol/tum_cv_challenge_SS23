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
    %% 1. Preprocessing
    % Convert the images to grayscale TODO: consider using color informaiton for the reconstruction
    I1_gray = rgb2gray(I1);
    I2_gray = rgb2gray(I2);

    % Remove lens distortion from the images
    I1_gray = undistortImage(I1_gray, camera_params);
    I2_gray = undistortImage(I2_gray, camera_params);

    % TODO: Apply a canny detector to the images to get the edges of the objects in the images
    % I1_gray_canny = edge(I1_gray, 'Canny');
    % I2_gray_canny = edge(I2_gray, 'Canny');

    % TODO: Take a look at Hough transformation (i.e Fourier transform) to get lines in the images

    % Detect features in the images 
    % TODO: consider changing detection algorithm and getting different point types
    % https://de.mathworks.com/help/vision/ug/point-feature-types.html
    
    %% 2. Feature detection and matching
    points1 = detectMinEigenFeatures(I1_gray, MinQuality = 0.1);
    points2 = detectMinEigenFeatures(I2_gray, MinQuality = 0.1);

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

    %% 3. Relative pose estimation
    relPose = estrelpose(E, camera_params.Intrinsics, camera_params.Intrinsics, inlier_points1, inlier_points2);
    if debugging
        disp("Relative pose is ");
        disp(relPose);
    end

    % Recompute matched points but only using points that are in the middle of the image
    % TODO: try to get a lot of points so that we can use more points for the triangulation and get a larger point cloud
    border = 200;  % TODO: consider changing this value, dont hard code it
    roi = [border, border, size(I1_gray, 2) - 2 * border, size(I1_gray, 1) - 2 * border];
    points1 = detectMinEigenFeatures(I1_gray, 'ROI', roi, MinQuality = 0.05);
    roi = [border, border, size(I2_gray, 2) - 2 * border, size(I2_gray, 1) - 2 * border];   
    points2 = detectMinEigenFeatures(I2_gray, 'ROI', roi, MinQuality = 0.05);
    [features1, valid_points1] = extractFeatures(I1_gray, points1);
    [features2, valid_points2] = extractFeatures(I2_gray, points2);
    index_pairs = matchFeatures(features1, features2);
    matched_points1 = valid_points1.Location(index_pairs(:, 1), :);
    matched_points2 = valid_points2.Location(index_pairs(:, 2), :);
    % UNCOMMENT IF DEBUGGING
    figure;
    showMatchedFeatures(I1, I2, matched_points1, matched_points2);
    legend("matched points 1","matched points 2");

    % Compute the 3D points from the camera pose
    camMatrix1 = cameraProjection(camera_params.Intrinsics, rigidtform3d);
    camMatrix2 = cameraProjection(camera_params.Intrinsics, pose2extr(relPose));
    [world_points, reprojection_error, valid_index] = triangulate(matched_points1, matched_points2, camMatrix1, camMatrix2);
    % TODO: Scale properly, I dont know the units of the camera matrix
    world_points = world_points / 10; 
    % TODO: do bundle adjustment

    if debugging
        disp("world_points size is ");
        disp(size(world_points));
        disp("reprojection_error size is ");
        disp(size(reprojection_error));
        disp("valid_index size is ");
        disp(size(valid_index));
    end
    % Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
    % remove points with high reprojection error
    max_reprojection_error = 100;
    max_z = 100;
    world_points = world_points(valid_index, :);  % TODO: filter better like described above
    disp(size(world_points))
    % UNCOMMENT IF DEBUGGING
    figure;
    hold on;
    % Show cameras pose
    plotCamera('Size', 0.05, 'Color', 'r', 'Label', '1');  % plot first camera
    plotCamera('Size', 0.05, 'Color', 'b', 'Label', '2', 'AbsolutePose', relPose);  % plot second camera
    % Show point cloud by coloring it based on the pixel color of the first image
    % Get the color of each reconstructed point
    numPixels = size(I1, 1) * size(I1, 2);
    allColors = reshape(I1, [numPixels, 3]);
    colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matched_points1(valid_index, 2)), round(matched_points1(valid_index, 1)));
    
    color = allColors(colorIdx, :);
    point_cloud = pointCloud(world_points, 'Color', color);

    pcshow(point_cloud,'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 45);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');

    % Rotate and zoom the plot
    camorbit(0, -30);
