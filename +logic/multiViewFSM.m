%% Step 1: Load the stereo images
% Make sure to download the images from moodle, before running this script
% The script is wip and not yet modularized
imageDir = 'C:\Users\mariu\Documents\UNI_MariusGhica\Master\Semester 2\Computer Vision\tum_3D_reconstruction\kicker_dslr_undistorted\kicker\images\dslr_images_undistorted';
imageFiles = dir(fullfile(imageDir, '*.jpg')); % Assuming JPEG format for the images
numImages = numel(imageFiles);
images = cell(1, numImages);

for i = 1:numImages
    imagePath = fullfile(imageDir, imageFiles(i).name);
    images{i} = imread(imagePath);
end

%% Step 2: Load camera parameters from cameras.txt
camera_params = logic.loadCameraParams('C:\Users\mariu\Documents\UNI_MariusGhica\Master\Semester 2\Computer Vision\tum_3D_reconstruction\kicker_dslr_undistorted\kicker\dslr_calibration_undistorted\cameras.txt');

%% Step 3: Pre-process the images
for i = 1:numImages
    [~, images{i}, ~] = logic.preprocessImage(images{i}, camera_params);
end 

%% Step 3: Detect and extract features
I1 = images{1};
[vSet, prevFeatures, prevPoints] = logic.createViewSet(I1, camera_params);

for i = 2:numImages
    % Load the current image
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

%% Step 4: Dense Reconstruction
I1 = images{1};
% Detect corners in the first image.
prevPoints = detectMinEigenFeatures(I1, MinQuality=0.001);

% Create the point tracker object to track the points across views.
tracker = vision.PointTracker(NumPyramidLevels=6);

% Initialize the point tracker.
prevPoints = prevPoints.Location;
initialize(tracker, prevPoints, I1);

% Store the dense points in the view set.

vSet = updateConnection(vSet, 1, 2, Matches=zeros(0, 2));
vSet = updateView(vSet, 1, Points=prevPoints);

% Track the points across all views.
for i = 2:numImages
    % Read and undistort the current image.
    I = images{i};
    
    % Track the points.
    [currPoints, validIdx] = step(tracker, I);
    
    % Clear the old matches between the points.
    if i < numImages
        vSet = updateConnection(vSet, i, i+1, Matches=zeros(0, 2));
    end
    vSet = updateView(vSet, i, Points=currPoints);
    
    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);        
    vSet = updateConnection(vSet, i-1, i, Matches=matches);
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);


% Triangulate initial locations for the 3-D world points.
[xyzPoints, ~, goodIdx] = triangulateMultiview(tracks, camPoses, camera_params.Intrinsics);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, camera_params.Intrinsics, FixedViewId=1, ...
    PointsUndistorted=true);
goodIdx = goodIdx  & (xyzPoints(:, 3) > 0) & (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);
plotting.plotPointCloud(xyzPoints, camPoses{:,2});
