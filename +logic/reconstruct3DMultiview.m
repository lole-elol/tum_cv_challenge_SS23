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
p.addOptional('scalingFactor', 0.1);
%% Reconstruction Params
% Feature extraction parameters
p.addOptional('featureExtractionMethod', 'SURF');
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
featureExtractionMethod = p.Results.featureExtractionMethod;
numOctaves = p.Results.numOctaves;
roiBorder = p.Results.roiBorder;
eMaxDistance = p.Results.eMaxDistance;
eConfidence = p.Results.eConfidence;
eMaxNumTrials = p.Results.eMaxNumTrials;
eValidPointFraction = p.Results.eValidPointFraction;
maxReprojectionError = p.Results.maxReprojectionError;
% =========================

imagesOriginal = images;
numImages = length(images);
tic;
if log
    fprintf('Starting preprocessing\n');
end
%% === 1. Preprocessing ===
for i = 1:numImages
    if log
        fprintf('Analyzing image %d of %d\r', i, numImages);
    end
    imagesOriginal{i} = imresize(imagesOriginal{i}, scalingFactor);
    images{i} = logic.reconstruct3D.preprocessImage(imagesOriginal{i});
end 
% Modify camera parameters to compensate for image resizing
cameraParams = logic.reconstruct3D.scaleCameraParameters(cameraParams, scalingFactor, size(images{1}));

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


for i = 2:numImages
    if log
        fprintf('Matching points of image %d of %d and triangulation with previous images. \r', i, numImages);
    end
    % Match the features between the previous and the current image.
    indexPairs   = matchFeatures(features{i-1}, features{i}, 'Unique', true);
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
    [pointCloudInstance, camPoses] = logic.reconstruct3D.getTriangulatedPointsMultiView(tracks, camPoses, cameraParams, ...
                                                                                        maxReprojectionError=maxReprojectionError);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);
end

minZ = min(pointCloudInstance.Location(:, 3))
maxZ = max(pointCloudInstance.Location(:, 3)) 
% We will use this to filter out points that are too far away.
% after doing dense reconstruction.


if log
    fprintf('\n3D reconstruction finished after %.2f seconds.\n', toc);
end


% === 4. Dense reconstruction ===
if log
    fprintf('Dense reconstruction\n');
end
% Create a dense point cloud from the triangulated points and camera poses.
for i = 1:1
    if log
        fprintf('Dense reconstruction of image %d of %d \r', i, numImages-1);
    end
    image1 = imagesOriginal{i};
    image2 = imagesOriginal{i+1};
    figure
    imshowpair(image1, image2, 'montage');
    
    % Compute stereo parameters and rectify the stereo images.
    relPose = rigidtform3d(poses(vSet, i+1).AbsolutePose.A);
    stereoParams = stereoParameters(cameraParams, cameraParams, relPose);
    %[image1Rect, image2Rect] = rectifyStereoImages(rgb2gray(image1), rgb2gray(image2), stereoParams);
    [image1Rect, image2Rect] = rectifyStereoImages(image1, image2, stereoParams);
    imshowpair(image1Rect, image2Rect, 'montage');
    
    % Compute disparity.
    disparityMap = disparitySGM(rgb2gray(image1Rect), rgb2gray(image2Rect));
    % Remove left most column of zeros from disparity map and corresponding pixels from the rectified image.
    figure
    % imshow(disparityMap);
    % colormap jet
    % colorbar

    % Reconstruct the 3-D world coordinates of points corresponding to each pixel from the disparity map.
    xyzPoints = reconstructScene(disparityMap, stereoParams);
    numColumnsToRemove = 200;
    xyzPoints = xyzPoints(:, numColumnsToRemove:end, :);
    disparityMap = disparityMap(:, numColumnsToRemove:end);
    % imshow(disparityMap);

    
    % Filter points that are too far away and transform the MxNx3 matrix into a Nx3 matrix.
    xyzPoints = reshape(xyzPoints, [], 3);
    pixelColors = reshape(image1Rect, [], 3);
    size(xyzPoints)
    size(pixelColors)
    validIdx = xyzPoints(:, 3) < maxZ & xyzPoints(:, 3) > minZ;
    xyzPoints = xyzPoints(validIdx, :);
    pixelColors = pixelColors(validIdx, :);


    % create a pointCloud object and assing the color of the original image at the corresponding pixel
    size(xyzPoints)
    size(pixelColors)
    % pointCloudInstance = pointCloud(xyzPoints, Color=pixelColors);
    pointCloudInstance = pointCloud(xyzPoints);
end
if log
    fprintf('\nDense reconstruction finished after %.2f seconds.\n', toc);
end

% Rotate the point cloud
R = [1 0 0; 0 0 1; 0 -1 0];
tform = affinetform3d([R, zeros(3, 1); zeros(1, 3), 1]);
[pointCloudInstance, camPoses] = logic.reconstruct3D.transformScene(pointCloudInstance, camPoses, tform);

end