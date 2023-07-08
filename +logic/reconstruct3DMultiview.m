function [pointCloudInstance, camPoses, tracks] = reconstruct3DMultiview(images, cameraParams, varargin)
% RECONSTRUCT3DMULTIVIEW Reconstructs a 3D point cloud from multiple images
% Input:
%   images - a cell array of images
%   cameraParams - a cameraParameters object
%   log - a boolean indicating whether to log the progress
%   scalingFactor - a factor by which the images are scaled down
%   numOctaves - the number of octaves used for feature extraction
%   roiBorder - the border around the image that is ignored for feature extraction
%   eMaxDistance - the maximum distance from an epipolar line for a point to be considered an inlier
%   eConfidence - the confidence level for the epipolar geometry
%   eMaxNumTrials - the maximum number of trials for the epipolar geometry
%   eValidPointFraction - the minimum fraction of points that must be in front of both cameras for the epipolar geometry to be valid
%   maxReprojectionError - the maximum reprojection error for a point to be considered valid
%   presort - the type of presorting to use. Options are: FFT2, HIST, PCA
%   presortNearestNeighbors - whether to presort by nearest neighbors or by 1D features
%   presortFeatures - the length of the feature vector
%   presortNormalize - whether to normalize the features before presorting
% Output:
%   pointCloudInstance - a pointCloud object
%   camPoses - a table containing the camera poses
%   tracks - a table containing the point tracks

%% === 0. Parse input ===
p = inputParser;
p.addOptional('log', true);
%% Preprocessing Params
p.addOptional('scalingFactor', 0.5);
%% Reconstruction Params
% Feature extraction parameters
p.addOptional('featureExtractionMethod', 'SURF');
p.addOptional('numOctaves', 20);
p.addOptional('roiBorder', 2);
% Epipolar geometry parameters
p.addOptional('eMaxDistance', 5);
p.addOptional('eConfidence', 99.6);
p.addOptional('eMaxNumTrials', 100000);
p.addOptional('eValidPointFraction', 0.8);
% Triangulation parameters
p.addOptional('maxReprojectionError', 20);
% Presorting parameters
p.addOptional('presort', 'HIST');
p.addOptional('presortNearestNeighbors', true);
p.addOptional('presortFeatures', 1);
p.addOptional('presortNormalize', true);

p.parse(varargin{:});
log = p.Results.log;
scalingFactor = p.Results.scalingFactor;
featureExtractionMethod = p.Results.featureExtractionMethod;
numOctaves = p.Results.numOctaves;
roiBorder = p.Results.roiBorder;
eMaxDistance = p.Results.eMaxDistance;
eConfidence = p.Results.eConfidence;
eMaxNumTrials = p.Results.eMaxNumTrials;
eValidPointFraction = p.Results.eValidPointFraction;
maxReprojectionError = p.Results.maxReprojectionError;
presort = p.Results.presort;
presortNearestNeighbors = p.Results.presortNearestNeighbors;
presortFeatures = p.Results.presortFeatures;
presortNormalize = p.Results.presortNormalize;
% =========================

numImages = length(images);
tic;
if log
    fprintf('Starting preprocessing\n');
end
%% === 1. Preprocessing ===
imagesOriginal = cell(numImages, 1);
for i = 1:numImages
    if log
        fprintf('Analyzing image %d of %d\r', i, numImages);
    end
    images{i} = imresize(images{i}, scalingFactor);
    imagesOriginal{i} = images{i};  % Save original image for later use
    images{i} = logic.reconstruct3D.preprocessImage(images{i});
end
% Modify camera parameters to compensate for image resizing
cameraParams = logic.reconstruct3D.scaleCameraParameters(cameraParams, scalingFactor, [size(images{1},1), size(images{1},2)]);

% Presort images
% fprintf('Presorting images\n');
% images = logic.presort(images, featureLength=presortFeatures, featureType=presort, normalize=presortNormalize, sortNearestNeighbors=presortNearestNeighbors);

if log
    fprintf('\nPreprocessing finished in %f seconds.\n', toc);
end
% =========================

%% === 2. Extract Features ===
% We first extract the features/points of all the images and save them on a cell
% array. This is done to avoid recomputing the features when matching the
% images.
if log
    fprintf('Extracting features\n');
end
features = cell(numImages, 1);
points = cell(numImages, 1);
for i = 1:numImages
    if log
        fprintf('Extracting features of image %d of %d\r', i, numImages);
    end
    currentImage = images{i};
    [points{i}, features{i}] = logic.reconstruct3D.extractFeatures(currentImage, ...
                                                                   method=featureExtractionMethod, ...
                                                                   numOctaves=numOctaves, roiBorder=roiBorder);
end
if log
    fprintf('\nFeature extraction finished in %f seconds.\n', toc);
end


matchThreshold = 0.7;
loopClosureThreshold = 2500;
% Initialize loop closure detection variables
loopClosureDetected = false;
loopClosureImageIndices = [];

for i = 1:numImages
    % Get the features and points of the current image
    currentFeatures = features{i};
    currentPoints = points{i};
    
    % Perform feature matching with previous images
    for j = 1:i-1
        % Get the features and points of the previous image
        previousFeatures = features{j};
        previousPoints = points{j};
        
        % Perform feature matching
        indexPairs = matchFeatures(currentFeatures, previousFeatures, 'MatchThreshold', matchThreshold);
        
        % Check if loop closure is detected
        numMatches = size(indexPairs, 1);
        if numMatches >= loopClosureThreshold
            loopClosureDetected = true;
            loopClosureImageIndices = [j, i];  % Store the indices of the loop closure images
            break;  % Exit the loop if loop closure is detected
        end
    end
    
    if loopClosureDetected
        break;  % Exit the loop if loop closure is detected
    end
end

if loopClosureDetected
    fprintf('Loop closure detected between images %d and %d\n', loopClosureImageIndices(1), loopClosureImageIndices(2));
else
    fprintf('No loop closure detected\n');
end

% === 3. Feature Matching and triangulation ===
% The object vSet contains the view set that stores the views (camera pose, feature vectors and points) and the connections
% (relative poses and point matches) between the views.
if log
    fprintf("Generating view set\n");
end
% Create an empty imageviewset object to manage the data associated with each view.
% Add the first view. Place the camera associated with the first view and the origin, oriented along the Z-axis.
vSet = imageviewset;
vSet = addView(vSet, 1, rigidtform3d, Points=points{1});

% matchedPointsLoopClosure = []; % Matched points for loop closure
% matchedPointsCurrent = []; % Matched points in the current keyframe
% loopClosureIndex = 0;
% loopClosureThreshhold = 200; % Threshold for loop closure detection
% matchedPointsCounts = zeros(numImages-1, 1); % Number of matched points for each image
% 
% % Loop over the images and find closures
% for i = 2:numImages
%     indexPairs = matchFeatures(features{i-1}, features{i}, 'Unique', true);
% 
%     % Count the number of matched features
%     matchedPointsCount = size(indexPairs, 1);
%     matchedPointsCounts(i-1) = matchedPointsCount;
% 
%     % Check if loop closure is detected based on the number of matched features
%     if size(indexPairs, 1) > loopClosureThreshhold || (i == 31)
%         loopClosureIndex = i;
%         matchedPointsLoopClosure = points{i}(indexPairs(:, 2));
%         matchedPointsCurrent = points{i-1}(indexPairs(:, 1));
%         break;
%     end 
% end
% 
% figure;
% plot(2:numImages, matchedPointsCounts);
% xlabel('Keyframe Index');
% ylabel('Number of Matched Points');
% title('Number of Matched Points in the Loop over Keyframes');
% grid on;

for i = 2:numImages
    if log
        fprintf('Matching points of image %d of %d and triangulation with previous images. \r', i, numImages);
    end
    % Match the features between the previous and the current image.
    indexPairs = matchFeatures(features{i-1}, features{i}, 'Unique', true);
    matchedPointsPrev = points{i-1}(indexPairs(:, 1));
    matchedPointsCurr = points{i}(indexPairs(:, 2));

    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [E, relPose, status, inlierIdx] = logic.reconstruct3D.getEpipolarGeometry(matchedPointsPrev, matchedPointsCurr, cameraParams, ...
                                                                              eMaxDistance=eMaxDistance, eConfidence=eConfidence, eMaxNumTrials=eMaxNumTrials, eValidPointFraction=eValidPointFraction);


    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1).AbsolutePose;
    % Compute the current camera pose in the global coordinate system
    % relative to the first view.
    if length(relPose) > 1  % TODO: BUG sometimes two poses are returned. This is a workaround.
        size(relPose)
        relPose = relPose(1);
    end
    currPose = rigidtform3d(prevPose.A * relPose.A);

    % Add the current view to the view set.
    vSet = addView(vSet, i, currPose, Points=points{i});
    % Store the point matches between the previous and the current views.

    vSet = addConnection(vSet, i-1, i, relPose, Matches=indexPairs(inlierIdx, :));
    
    tracks = findTracks(vSet);  % Find point tracks across all views.
    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points and do bundle adjustment.
    [worldPoints, camPoses, tracks] = logic.reconstruct3D.getTriangulatedPointsMultiView(tracks, camPoses, cameraParams, ...
                                                                                        maxReprojectionError=maxReprojectionError);
    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);
end

figure;
plotCamera(camPoses, Size=0.2);

figure;
plotCamera(camPoses, Size=0.2);
hold on;
pointCloudColored = logic.reconstruct3D.getColoredPointCloud(worldPoints, tracks, imagesOriginal);
pcshow(pointCloudColored, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 45);

if i == 34
    if log
        fprintf('Loop closure detected between images %d and %d\n', 1, i);
    end
    indexPairs = matchFeatures(features{1}, features{i}, 'Unique', true);
    matchedPointsPrev = points{1}(indexPairs(:, 1));
    matchedPointsCurr = points{i}(indexPairs(:, 2));
    [E, relPose, status, inlierIdx] = logic.reconstruct3D.getEpipolarGeometry(matchedPointsPrev, matchedPointsCurr, cameraParams, ...
                                                                              eMaxDistance=eMaxDistance, eConfidence=eConfidence, eMaxNumTrials=eMaxNumTrials, eValidPointFraction=eValidPointFraction);
    prevPose = poses(vSet, 1).AbsolutePose;
    currPose = rigidtform3d(prevPose.A * relPose.A);
    vSet = addConnection(vSet, 1, i, Matches=indexPairs(inlierIdx, :));
    vSet = optimizePoses(vSet);

    tracks = findTracks(vSet);  % Find point tracks across all views.
    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points and do bundle adjustment.
    [worldPoints, camPoses, tracks] = logic.reconstruct3D.getTriangulatedPointsMultiView(tracks, camPoses, cameraParams, ...
                                                                                        maxReprojectionError=maxReprojectionError);
    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);
end
figure;
plotCamera(camPoses, Size=0.2);
figure;
plotCamera(camPoses, Size=0.2);
hold on;
pointCloudColored = logic.reconstruct3D.getColoredPointCloud(worldPoints, tracks, imagesOriginal);
pcshow(pointCloudColored, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 45);

if log
    fprintf('\n3D reconstruction finished after %.2f seconds.\n', toc);
end

% Rotate the point cloud
R = [1 0 0; 0 0 1; 0 -1 0];
tform = affinetform3d([R, zeros(3, 1); zeros(1, 3), 1]);
[pointCloudInstance, camPoses] = logic.reconstruct3D.transformScene(pointCloudColored, camPoses, tform);

if log
    fprintf('\n3D reconstruction finished after %.2f seconds.\n', toc);
end

end