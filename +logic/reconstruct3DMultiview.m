function [pointCloudInstance, camPoses, tracks] = reconstruct3DMultiview(images, cameraParams, varargin)
% RECONSTRUCT3DMULTIVIEW Reconstructs a 3D point cloud from multiple images
% Input:
%   images - a cell array of images
%   cameraParams - a cameraParameters object
% Output:
%   pointCloudInstance - a pointCloud object
%   camPoses - a table containing the camera poses
%   tracks - a table containing the point tracks

p = inputParser;
p.addOptional('log', true);
p.parse(varargin{:});
log = p.Results.log;

numImages = length(images);

if log
    fprintf('Starting preprocessing\n');
end
%% === 1. Preprocessing ===
for i = 1:numImages
    if log
        fprintf('Analyzing image %d of %d\r', i, numImages);
    end
    % TODO: resampling leads to shear reconstruction results. Cam parameters need to be adjusted (if that works)
    images{i} = imresize(images{i}, 1.0);
    % TODO: for now we only use the grayscale image, but we should use the output of the preprocessImage function
    % [~, images{i}, ~] = logic.reconstruct3D.preprocessImage(images{i});
    images{i} = im2gray(images{i});
end 
% =========================
if log
    fprintf('\nPreprocessing finished.\n');
end

%% === 2. Feature Extraction First Image ===
image1 = images{1};
% The object vSet contains the view set that stores the views (camera pose, feature vectors and points) and the connections
% (relative poses and point matches) between the views.
if log
    tic;
    fprintf("Generating view set and finding features of first picture\n");
end
[vSet, prevFeatures, prevPoints] = logic.reconstruct3D.createViewSet(image1);

% === 3. Feature Extraction and matching remaining Images ===
for i = 2:numImages
    if log
        fprintf('Matching points of image %d of %d\r', i, numImages);
    end
    % Load the current image and extract common features to all previous views.
    currentImage = images{i};

    [matchedPoints1, matchedPoints2, currPoints, currFeatures, indexPairs] = logic.reconstruct3D.extractCommonFeaturesMultiView(currentImage, prevFeatures, prevPoints);
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [E, relPose, status, inlierIdx] = logic.reconstruct3D.getEpipolarGeometry(matchedPoints1, matchedPoints2, cameraParams);
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1).AbsolutePose;
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    if length(relPose) > 1  % TODO: BUG sometimes two poses are returned. This is a workaround
        size(relPose)
        relPose = relPose(1);
    end
    currPose = rigidtform3d(prevPose.A*relPose.A); 
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, currPose, Points=currPoints);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, relPose, Matches=indexPairs(inlierIdx,:));
    
    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    [pointCloudInstance, camPoses] = logic.reconstruct3D.getTriangulatedPointsMultiView(tracks, camPoses, cameraParams);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints = currPoints;  
end
if log
    fprintf('\n3D reconstruction finished after %.2f seconds.\n', toc);
end
end