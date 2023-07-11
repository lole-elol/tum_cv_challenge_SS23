function [pointCloudInstance, camPoses, tracks] = reconstruct3DMultiview(images, cameraParams, varargin)
% RECONSTRUCT3DMULTIVIEW Reconstructs a 3D point cloud from multiple images
%
% Input:
%   images: a cell array of images
%   cameraParams: a cameraParameters object
%   log = true: a boolean indicating whether to log the progress
%   scalingFactor = 0.5: a factor by which the images are scaled down
%   numOcatves = 20: the number of octaves used for feature extraction
%   roiBorder = 2: the border around the image that is ignored for feature extraction
%   eMaxDistance = 5: the maximum distance from an epipolar line for a point to be considered an inlier
%   eConfidence = 99.6: the confidence level for the epipolar geometry
%   eMaxNumTrials = 5000: the maximum number of trials for the epipolar geometry
%   eValidPointFraction = 0.8: the minimum fraction of points that must be in front of both cameras for the epipolar geometry to be valid
%   maxReprojectionError = 20: the maximum reprojection error for a point to be considered valid
%   presort = 'FFT2': the type of presorting to use. Options are: FFT2, HIST, PCA
%   presortFeatures = 1: the length of the feature vector
%   presortNormalize = true: whether to normalize the features before presorting
%   presortLazy = true: whether to use lazy presorting
%   progressdlg = []; a progress dialog object to update
%   progressdlgMax = 0.6: the maximum value of the progress dialog
%
% Output:
%   pointCloudInstance: a pointCloud object
%   camPoses: a table containing the camera poses
%   tracks: a table containing the point tracks

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
p.addOptional('eMaxNumTrials', 5000);
p.addOptional('eValidPointFraction', 0.8);
% Triangulation parameters
p.addOptional('maxReprojectionError', 20);
% Presorting parameters
p.addOptional('presort', 'FFT2');
p.addOptional('presortFeatures', 1);
p.addOptional('presortNormalize', true);
% Progress dialog
p.addOptional('progressdlg', []);
p.addOptional('progressdlgMax', 0.6);

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
presortFeatures = p.Results.presortFeatures;
presortNormalize = p.Results.presortNormalize;
progressdlg = p.Results.progressdlg;
progressdlgMax = p.Results.progressdlgMax;
% =========================

numImages = length(images);
progressdlgMaxPreprocessing = progressdlgMax * 0.2;
progressdlgMaxFeatureExtraction = progressdlgMax * 0.4;
progressdlgMaxMatchingTriangulation = progressdlgMax * 0.4;

tic;
if ~isempty(progressdlg)
    progressdlg.Value = 0;
    progressdlg.Message = 'Preprocessing images';
end
if log
    fprintf('Starting preprocessing\n');
end
%% === 1. Preprocessing ===
imagesOriginal = cell(1, numImages);
for i = 1:numImages
    if ~isempty(progressdlg)
        progressdlg.Message = sprintf('Preprocessing image %d of %d', i, numImages);
        progressdlg.Value = (i-1)/numImages * progressdlgMaxPreprocessing;
    end
    if log
        fprintf('Analyzing image %d of %d\r', i, numImages);
    end
    images{i} = imresize(images{i}, scalingFactor);
    imagesOriginal{i} = images{i};  % Save original image for later use
    images{i} = logic.reconstruct3D.preprocessImage(images{i});
end
% Modify camera parameters to compensate for image resizing
cameraParams = logic.reconstruct3D.scaleCameraParameters(cameraParams, scalingFactor, [size(images{1},1), size(images{1},2)]);

% Compute similarity matrix
if ~isempty(progressdlg)
    progressdlg.Message = 'Computing similarity matrix';
    progressdlg.Value = progressdlgMaxPreprocessing;
end
fprintf('Computing similarity matrix\n')
imagesLowRes = cellfun(@(x) im2gray(imresize(x, 0.1)), imagesOriginal, 'UniformOutput', false);
[~, similarityMatrix] = logic.reconstruct3D.similarity(imagesLowRes, featureLength=presortFeatures, featureType=presort, normalize=presortNormalize);

if ~isempty(progressdlg)
    progressdlg.Message = sprintf('Preprocessing finished in %f seconds', toc);
    progressdlg.Value = progressdlgMaxPreprocessing;
end
if log
    fprintf('\nPreprocessing finished in %f seconds.\n', toc);
end
% =========================

%% === 2. Extract Features ===
% We first extract the features/points of all the images and save them on a cell
% array. This is done to avoid recomputing the features when matching the
% images.
tic;
if ~isempty(progressdlg)
    progressdlg.Message = 'Extracting features';
    progressdlg.Value = progressdlgMaxPreprocessing;
end
if log
    fprintf('Extracting features\n');
end
features = cell(numImages, 1);
points = cell(numImages, 1);
for i = 1:numImages
    if ~isempty(progressdlg)
        progressdlg.Message = sprintf('Extracting features of image %d of %d', i, numImages);
        progressdlg.Value = progressdlgMaxPreprocessing + progressdlgMaxFeatureExtraction * (i-1)/numImages;
    end
    if log
        fprintf('Extracting features of image %d of %d\r', i, numImages);
    end
    currentImage = images{i};
    [points{i}, features{i}] = logic.reconstruct3D.extractFeatures(currentImage, ...
                                                                   method=featureExtractionMethod, ...
                                                                   numOctaves=numOctaves, roiBorder=roiBorder);
end
if ~isempty(progressdlg)
    progressdlg.Message = sprintf('Feature extraction finished in %f seconds', toc);
    progressdlg.Value = progressdlgMaxPreprocessing + progressdlgMaxFeatureExtraction;
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

prevIdx = 1;
similarityMatrix(:, prevIdx) = inf;

tic;
for i=2:numImages
    if ~isempty(progressdlg)
        progressdlg.Message = sprintf('Matching points of image %d of %d and triangulation with previous images', i, numImages);
        progressdlg.Value = progressdlgMaxPreprocessing + progressdlgMaxFeatureExtraction + progressdlgMaxMatchingTriangulation * (i-1)/numImages * progressdlgMax;
    end
    if log
        fprintf('Matching points of image %d of %d and triangulation with previous images. \n', i, numImages);
    end

    k = 0;
    status = 1;
    while status ~= 0
        % Find the best match between for previous image.

        if k == 0 && prevIdx + 1 < numImages && similarityMatrix(prevIdx, prevIdx+1) ~= inf
            % Try to match the next image
            currIdx = prevIdx + 1;
        else
            if k == 0
                k = 1;
            end

            % Find the most similar image
            currIdxTmp = logic.reconstruct3D.kNearestNeighbour(similarityMatrix(prevIdx, :), k);

            % If there are no more images to match use the previous result
            if similarityMatrix(prevIdx, currIdxTmp) == inf
                if log
                    fprintf('No more images to match\n');
                end
                break;
            end

            currIdx = currIdxTmp;
        end

        if log
            fprintf('Matching image %d to %d\n', prevIdx, currIdx);
        end

        % Match the features between the previous and the current image.
        indexPairs   = matchFeatures(features{prevIdx}, features{currIdx}, 'Unique', true);
        matchedPointsPrev = points{prevIdx}(indexPairs(:, 1));
        matchedPointsCurr = points{currIdx}(indexPairs(:, 2));

        % Estimate the camera pose of current view relative to the previous view.
        % The pose is computed up to scale, meaning that the distance between
        % the cameras in the previous view and the current view is set to 1.
        % This will be corrected by the bundle adjustment.
        [E, relPose, status, inlierIdx] = logic.reconstruct3D.getEpipolarGeometry(matchedPointsPrev, matchedPointsCurr, cameraParams, ...
                                                                                eMaxDistance=eMaxDistance, eConfidence=eConfidence, eMaxNumTrials=eMaxNumTrials, eValidPointFraction=eValidPointFraction);

        k = k + 1;
    end

    % Update the similarity matrix
    similarityMatrix(:, prevIdx) = inf;

     % Get the table containing the previous camera pose.
    prevPose = poses(vSet, prevIdx).AbsolutePose;
    % Compute the current camera pose in the global coordinate system
    % relative to the first view.
    if length(relPose) > 1  % TODO: BUG sometimes two poses are returned. This is a workaround.
        relPose = relPose(1);
    end
    currPose = rigidtform3d(prevPose.A * relPose.A);

    try
        % Add the current view to the view set.
        vSet = addView(vSet, currIdx, currPose, Points=points{currIdx});
        % Store the point matches between the previous and the current views.
        vSet = addConnection(vSet, prevIdx, currIdx, relPose, Matches=indexPairs(inlierIdx, :));
        tracks = findTracks(vSet);  % Find point tracks across all views.
        % Get the table containing camera poses for all views.
        camPoses = poses(vSet);

        % Triangulate initial locations for the 3-D world points and do bundle adjustment.
        [worldPoints, camPoses, tracks] = logic.reconstruct3D.getTriangulatedPointsMultiView(tracks, camPoses, cameraParams, ...
                                                                                            maxReprojectionError=maxReprojectionError);
        % Store the refined camera poses.
        vSet = updateView(vSet, camPoses);

        prevIdx = currIdx;
    catch
        fprintf('Error triangulating points. Skipping image %d\n', currIdx);
    end
end

pointCloudInstance = logic.reconstruct3D.getColoredPointCloud(worldPoints, tracks, imagesOriginal);

% Rotate the point cloud
R = [1 0 0; 0 0 1; 0 -1 0];
tform = affinetform3d([R, zeros(3, 1); zeros(1, 3), 1]);
[pointCloudInstance, camPoses] = logic.reconstruct3D.transformScene(pointCloudInstance, camPoses, tform);

if ~isempty(progressdlg)
    progressdlg.Message = sprintf('Triangulation finished after %.2f seconds', toc);
    progressdlg.Value = progressdlgMaxPreprocessing + progressdlgMaxFeatureExtraction + progressdlgMaxMatchingTriangulation;
end

if log
    fprintf('\n3D reconstruction finished after %.2f seconds.\n', toc);
end

end