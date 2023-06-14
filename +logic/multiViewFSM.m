%% Step 1: Load the stereo images
% Make sure to download the images from moodle, before running this script
% The script is wip and not yet modularized
imageDir = 'delivery_area\images\dslr_images_undistorted';
imageFiles = dir(fullfile(imageDir, '*.jpg')); % Assuming JPEG format for the images
numImages = numel(imageFiles);
images = cell(1, numImages);

for i = 1:numImages
    imagePath = fullfile(imageDir, imageFiles(i).name);
    images{i} = imread(imagePath);
end
%% Step 2: Load camera parameters from cameras.txt
camera_params = loadCameraParams('delivery_area\dslr_calibration_undistorted\cameras.txt');

%% Step 3: Detect and extract features
% No pre-processing has been done on the images
I1 = images{1};
border = 20; % TODO: check if this is a good value
roi = [border, border, size(I1, 2)- 2*border, size(I1, 1)- 2*border];
prevPoints = detectSURFFeatures(im2gray(I1), NumOctaves=12, ROI=roi);
prevFeatures = extractFeatures(im2gray(I1), prevPoints);

% Create an empty imageviewset object to manage the data associated with each view.
vSet = imageviewset;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, rigidtform3d, Points=prevPoints);


for i = 2:numImages
    % Load the current image
    I = images{i};
    
    % Detect, extract and match features.
    currPoints   = detectSURFFeatures(im2gray(I), NumOctaves=12, ROI=roi);
    currFeatures = extractFeatures(im2gray(I), currPoints);    
    indexPairs   = matchFeatures(prevFeatures, currFeatures);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relPose, inlierIdx] = helperEstimateRelativePose(matchedPoints1, matchedPoints2, camera_params);
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1).AbsolutePose;
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    currPose = rigidtform3d(prevPose.A*relPose.A); 
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, currPose, Points=currPoints);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, relPose, Matches=indexPairs(inlierIdx,:));
    
    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, camera_params);
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, tracks, camPoses, camera_params);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints = currPoints;  
end

%% Step 4: Display camera poses
% camPoses = poses(vSet);
figure;
% plotCamera(camPoses, Size=0.2);
% hold on

%% Step 5: Exclude noisy 3-D points.
% goodIdx = (reprojectionErrors < 10); % TODO: check if this is a good threshold
% xyzPoints = xyzPoints(goodIdx, :);
% xyzPoints = xyzPoints(xyzPoints<30);


%% Step 5: Display the 3-D points.
% Issue: the points are not displayed at the correct angle? 
pcshow(xyzPoints, VerticalAxis='y', VerticalAxisDir='down', MarkerSize= 45);
grid on
hold off

% loc1 = camPoses.AbsolutePose(1).Translation;
% xlim([loc1(1)-20, loc1(1)+20]);
% ylim([loc1(2)-20, loc1(2)+20]);
% zlim([loc1(3)-20, loc1(3)+20]);
% camorbit(0, -30);


%% Helper functions
% Estimate the relative pose between two views based on point correspondences
function [relPose, inlierIdx] = helperEstimateRelativePose(matchedPoints1, matchedPoints2, camera_params)
    if ~isnumeric(matchedPoints1)
        matchedPoints1 = matchedPoints1.Location;
    end
    
    if ~isnumeric(matchedPoints2)
        matchedPoints2 = matchedPoints2.Location;
    end
    
    % Estimate the essential matrix.   
    % TODO: Set the confidence value, max distance and max number of trials to reasonable values
    [E, inlierIdx] = estimateEssentialMatrix(matchedPoints1, matchedPoints2, camera_params, "Confidence", 95, "MaxDistance", 10, "MaxNumTrials", 100000);
    
    % Get the epipolar inliers.
    inlierPoints1 = matchedPoints1(inlierIdx, :);
    inlierPoints2 = matchedPoints2(inlierIdx, :);    
    
    % Compute the camera pose from the fundamental matrix. Use half of the points to reduce computation.
    [relPose, validPointFraction] = estrelpose(E, camera_params.Intrinsics, inlierPoints1, inlierPoints2);

    % validPointFraction is the fraction of inlier points that project in
    % front of both cameras. If the this fraction is too small, then the
    % fundamental matrix is likely to be incorrect.
    if validPointFraction > 0.8 % TODO: check if this is a good threshold
        return;
    end
    
end

% Load the camera parameters from the file
function [camera_parameters] = loadCameraParams(filename)
    file = fopen(filename,'r');
    formatSpec = '%d %s %d %d %f %f %f %f';
    
    % Read the camera information from the file
    header = textscan(file, '%s', 3, 'Delimiter', '\n');
    data = textscan(file, formatSpec, 'Delimiter', ' ');

    % Close the file
    fclose(file);

    % Extract the camera information from the read data
    camera_id = data{1};
    model = data{2};
    width = data{3};
    height = data{4};
    parameters = [data{5}, data{6}, data{7}, data{8}];

    % Display the camera information
    % fprintf('Camera ID: %d\n', camera_id);
    % % fprintf('Model: %s\n', model);
    % fprintf('Width: %d\n', width);
    % fprintf('Height: %d\n', height);
    % fprintf('Parameters: %.2f %.2f %.2f %.2f\n', parameters);

    k = [parameters(1), 0, parameters(3); 0, parameters(2), parameters(4); 0, 0, 1];
    % Make width and height double
    width = double(width);
    height = double(height);
    camera_parameters = cameraParameters("K",k,"ImageSize",[height,width]);
end