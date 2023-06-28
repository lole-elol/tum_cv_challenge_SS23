%% Step 1: Load the stereo images
% Make sure to download the images from moodle, before running this script
% The script is wip and not yet modularized
imageDir = 'C:\Users\mariu\Documents\UNI_MariusGhica\Master\Semester 2\Computer Vision\tum_3D_reconstruction\delivery_area\images\dslr_images_undistorted';
imageFiles = dir(fullfile(imageDir, '*.jpg')); % Assuming JPEG format for the images
numImages = numel(imageFiles);
images = cell(1, numImages);

for i = 1:numImages
    imagePath = fullfile(imageDir, imageFiles(i).name);
    images{i} = imread(imagePath);
end
%% Step 2: Load camera parameters from cameras.txt
camera_params = logic.loadCameraParams('C:\Users\mariu\Documents\UNI_MariusGhica\Master\Semester 2\Computer Vision\tum_3D_reconstruction\delivery_area\dslr_calibration_undistorted\cameras.txt');

%% Step 3: Pre-process the images
for i = 1:numImages
    images{i} = logic.resampleImage(images{i}, 1);
    [~, images{i}, ~] = logic.preprocessImage(images{i});
end 

%% Step 3: Detect and extract features
I1 = images{1};
[vSet, prevFeatures, prevPoints] = logic.createViewSet(I1, camera_params);

for i = 2:numImages
    % Load the current imagex
    I = images{i};

    [matchedPoints1, matchedPoints2, currPoints, currFeatures, indexPairs] = logic.extractCommonFeaturesMultiView(I, prevFeatures, prevPoints, camera_params);
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [E, relPose, inlierIdx] = logic.getEpipolarGeometry(matchedPoints1, matchedPoints2, camera_params);
    
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

    [world_points, camPoses] = logic.getTriangulatedPointsMultiView(tracks, camPoses, camera_params);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints = currPoints;  
end


%plotting.plotPointCloud(world_points, camPoses{:,2});
