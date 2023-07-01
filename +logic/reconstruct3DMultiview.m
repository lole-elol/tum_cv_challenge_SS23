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
p.addOptional('numOctaves', 20);
p.addOptional('roiBorder', 20);
% Epipolar geometry parameters
p.addOptional('eMaxDistance', 5);
p.addOptional('eConfidence', 99.6);
p.addOptional('eMaxNumTrials', 100000);
p.addOptional('eValidPointFraction', 0.8);
% Triangulation parameters
p.addOptional('maxReprojectionError', 20);

p.parse(varargin{:});
log = p.Results.log;
scalingFactor = p.Results.scalingFactor;
numOctaves = p.Results.numOctaves;
roiBorder = p.Results.roiBorder;
eMaxDistance = p.Results.eMaxDistance;
eConfidence = p.Results.eConfidence;
eMaxNumTrials = p.Results.eMaxNumTrials;
eValidPointFraction = p.Results.eValidPointFraction;
maxReprojectionError = p.Results.maxReprojectionError;
% =========================


numImages = length(images);

if log
    fprintf('Starting preprocessing\n');
end
%% === 1. Preprocessing ===
for i = 1:numImages
    if log
        fprintf('Analyzing image %d of %d\r', i, numImages);
    end
    images{i} = imresize(images{i}, scalingFactor);
    % TODO: for now we only use the grayscale image, but we should use the output of the preprocessImage function
    % [~, images{i}, ~] = logic.reconstruct3D.preprocessImage(images{i});
    images{i} = im2gray(images{i});
end
% Modify camera parameters to compensate for image resizing
cameraParams = logic.reconstruct3D.scaleCameraParameters(cameraParams, scalingFactor, size(images{1}));

if log
    fprintf('\nPreprocessing finished.\n');
end
% =========================

%% === 2. Feature Extraction First Image ===
image1 = images{1};
% The object vSet contains the view set that stores the views (camera pose, feature vectors and points) and the connections
% (relative poses and point matches) between the views.
if log
    tic;
    fprintf("Generating view set and finding features of first picture\n");
end
[vSet, prevFeatures, prevPoints] = logic.reconstruct3D.createViewSet(image1, numOctaves=numOctaves, roiBorder=roiBorder);

% === 3. Feature Extraction and matching remaining Images ===
for i = 2:numImages
    if log
        fprintf('Matching points of image %d of %d\r', i, numImages);
    end
    % Load the current image and extract common features to all previous views.
    currentImage = images{i};

    [matchedPoints1, matchedPoints2, currPoints, currFeatures, indexPairs] = logic.reconstruct3D.extractCommonFeaturesMultiView(currentImage, prevFeatures, prevPoints, ...
                                                                                                                                numOctaves=numOctaves, roiBorder=roiBorder);

    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [E, relPose, status, inlierIdx] = logic.reconstruct3D.getEpipolarGeometry(matchedPoints1, matchedPoints2, cameraParams, ...
                                                                              eMaxDistance=eMaxDistance, eConfidence=eConfidence, eMaxNumTrials=eMaxNumTrials, eValidPointFraction=eValidPointFraction);

    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1).AbsolutePose;

    % Compute the current camera pose in the global coordinate system
    % relative to the first view.
    if length(relPose) > 1  % TODO: BUG sometimes two poses are returned. This is a workaround
        size(relPose)
        relPose = relPose(1);
    end
    currPose = rigidtform3d(prevPose.A * relPose.A); % TODO: the cameras are pointing in the wrong direction.

    % Add the current view to the view set.
    vSet = addView(vSet, i, currPose, Points=currPoints);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, relPose, Matches=indexPairs(inlierIdx, :));

    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    [pointCloudInstance, camPoses] = logic.reconstruct3D.getTriangulatedPointsMultiView(tracks, camPoses, cameraParams, ...
                                                                                        maxReprojectionError=maxReprojectionError);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints = currPoints;

end

% Rotate the point cloud
R = [1 0 0; 0 0 1; 0 -1 0];
tform = rigid3d(R, [0 0 0]);
pointCloudInstance = pctransform(pointCloudInstance, tform);

%for cam=camPoses.AbsolutePose'
    %cam.R = R * cam.R;
    %cam.Translation = (R * cam.Translation')';
%end

% TODO: Add an extra step to generate more features once we get the full 3D reconstruction

if log
    fprintf('\n3D reconstruction finished after %.2f seconds.\n', toc);
end
end