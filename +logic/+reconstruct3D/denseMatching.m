function pointCloudDense = denseMatching(pointCloudInstance, images, camPoses, vSet, cameraParams, varargin), 
% DENSEMATCHING 3D reconstruction of a scene from a set of images.
%   pointCloudDense = DENSEMATCHING(pointCloudInstance, images, cameraParams)
%   reconstructs a 3D scene from a set of images. The images are assumed to
%   be taken with a stereo camera rig. The function returns a pointCloud
%   object containing the 3D coordinates of the scene.
%
% Inputs:
%   pointCloudInstance - pointCloud object containing the 3D coordinates of
%                        the scene
%   images     - cell array containing the images
%   camPoses           - camera poses
%   cameraParams       - cameraParameters object containing the camera
%                        parameters
%   log = true         - (optional) boolean indicating whether to print
%                        progress to the console
%   scalingFactor = 0.1- (optional) scaling factor for the point cloud
%                        coordinates
%   numImages = Inf    - (optional) number of images to use for the
%                        reconstruction
% Outputs:
%   pointCloudDense    - pointCloud object containing the 3D coordinates of
%                        the scene

p = inputParser;
addRequired(p, 'pointCloudInstance', @isobject);
addRequired(p, 'images', @iscell);
addRequired(p, 'camPoses', @isobject);
addRequired(p, 'vSet', @isobject);
addRequired(p, 'cameraParams', @isobject);
addOptional(p, 'log', true, @islogical);
addOptional(p, 'scalingFactor', 1.0, @isnumeric);
addOptional(p, 'numImages', Inf, @isnumeric);
addOptional(p, 'maxZ', 1.5, @isnumeric);
parse(p, pointCloudInstance, images, camPoses, vSet, cameraParams, varargin{:});
log = p.Results.log;
scalingFactor = p.Results.scalingFactor;
numImages = p.Results.numImages;
maxZ = p.Results.maxZ;

if log
    fprintf('Dense reconstruction\n');
end

if scalingFactor ~= 1
    for i = 1:numImages
        images{i} = imresize(images{i}, scalingFactor);
        % TODO: maybe remove
        % images{i} = logic.reconstruct3D.preprocessImage(images{i});
    end
    % Modify camera parameters to compensate for image resizing
    cameraParams = logic.reconstruct3D.scaleCameraParameters(cameraParams, scalingFactor, [size(images{1}, 1), size(images{1}, 2)]);
end

pointCloudDense = pointCloudInstance;

% Create a dense point cloud from the triangulated points and camera poses.
for i = 1:(numImages-1)
    if log
        fprintf('Dense reconstruction of image %d of %d \r', i, numImages-1);
    end
    image1 = images{i};
    image2 = images{i+1};
    % figure
    % imshowpair(image1, image2, 'montage');

    % Compute stereo parameters and rectify the stereo images.
    relPose = rigidtform3d(poses(vSet, i).AbsolutePose.A \ poses(vSet, i+1).AbsolutePose.A);
    stereoParams = stereoParameters(cameraParams, cameraParams, relPose);
    %[image1Rect, image2Rect] = rectifyStereoImages(rgb2gray(image1), rgb2gray(image2), stereoParams);
    [image1Rect, image2Rect, ~, camMatrix1] = rectifyStereoImages(image1, image2, stereoParams);
    % figure
    % imshowpair(image1Rect, image2Rect, 'montage');

    % Compute disparity.
    disparityRange = [0 128];
    disparityMap = disparitySGM(rgb2gray(image1Rect), rgb2gray(image2Rect), DisparityRange=disparityRange);
    % figure
    % imshow(disparityMap, disparityRange);
    % colormap jet
    % colorbar


    % Reconstruct the 3-D world coordinates of points corresponding to each pixel from the disparity map.
    xyzPoints = reconstructScene(disparityMap, stereoParams);

    % Filter points that are too far away and transform the MxNx3 matrix into a Nx3 matrix
    roiBorder = 50;  % TODO: make this a parameter
    roi = zeros(size(disparityMap));
    roi(roiBorder:end-roiBorder, roiBorder:end-roiBorder) = 1;
    xyzPoints = reshape(xyzPoints, [], 3);
    roi = reshape(roi, [], 1);
    % validIdx = xyzPoints(:, 3) < 1.5*maxZ & xyzPoints(:, 3) > 0.5*minZ & roi;
    validIdx = xyzPoints(:, 3) < maxZ & roi;
    xyzPoints = xyzPoints(validIdx, :);
    xyzPointsInImage1 = (camMatrix1 * [xyzPoints, ones(size(xyzPoints, 1), 1)]')';
    xyzPointsInImage1 = floor(xyzPointsInImage1(:, 2:-1:1) ./ xyzPointsInImage1(:, 3));
    pixelColors = impixel(image1Rect, xyzPointsInImage1(: , 1), xyzPointsInImage1(: , 2));


    % create a pointCloud object and assing the color of the original image at the corresponding pixel
    size(xyzPoints)
    pointCloudDense = pccat([pointCloudDense, pointCloud(xyzPoints, Color=pixelColors)]);
end
if log
    fprintf('\nDense reconstruction finished after %.2f seconds.\n', toc);
end

% intrinsics = cameraParams.Intrinsics;
% % Read and undistort the first image
% I = undistortImage(images{1}, intrinsics); 

% % Detect corners in the first image.
% prevPoints = detectMinEigenFeatures(I, MinQuality=0.001);

% % Create the point tracker object to track the points across views.
% tracker = vision.PointTracker(MaxBidirectionalError=1, NumPyramidLevels=6);

% % Initialize the point tracker.
% prevPoints = prevPoints.Location;
% initialize(tracker, prevPoints, I);

% % Store the dense points in the view set.

% vSet = updateConnection(vSet, 1, 2, Matches=zeros(0, 2));
% vSet = updateView(vSet, 1, Points=prevPoints);

% % Track the points across all views.
% for i = 2:numel(images)
%     % Read and undistort the current image.
%     I = undistortImage(images{i}, intrinsics); 
    
%     % Track the points.
%     [currPoints, validIdx] = step(tracker, I);
    
%     % Clear the old matches between the points.
%     if i < numel(images)
%         vSet = updateConnection(vSet, i, i+1, Matches=zeros(0, 2));
%     end
%     vSet = updateView(vSet, i, Points=currPoints);
    
%     % Store the point matches in the view set.
%     matches = repmat((1:size(prevPoints, 1))', [1, 2]);
%     matches = matches(validIdx, :);        
%     vSet = updateConnection(vSet, i-1, i, Matches=matches);
% end

% % Find point tracks across all views.
% tracks = findTracks(vSet);

% % Find point tracks across all views.
% camPoses = poses(vSet);

% % Triangulate initial locations for the 3-D world points.
% xyzPoints = triangulateMultiview(tracks, camPoses,...
%     intrinsics);

% % Refine the 3-D world points and camera poses.
% [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
%     xyzPoints, tracks, camPoses, intrinsics, FixedViewId=1, ...
%     PointsUndistorted=true);

% pointCloudDense = pointCloud(xyzPoints, Color=[0.8, 0.8, 0.8]);

end
