function pointCloudDense = denseMatching(pointCloudInstance, images, camPoses, cameraParams, varargin), 
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
addRequired(p, 'cameraParams', @isobject);
addOptional(p, 'log', true, @islogical);
addOptional(p, 'scalingFactor', 1, @isnumeric);
addOptional(p, 'numImages', Inf, @isnumeric);
addOptional(p, 'maxZ', 1.5, @isnumeric);
parse(p, pointCloudInstance, images, camPoses, cameraParams, varargin{:});
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
    relPose = rigidtform3d(camPoses.AbsolutePose(i).A \ camPoses.AbsolutePose(i+1).A);
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
    roiBorder = 1;  % TODO: make this a parameter
    roi = zeros(size(disparityMap));
    roi(roiBorder:end-roiBorder, roiBorder:end-roiBorder) = 1;
    xyzPoints = reshape(xyzPoints, [], 3);
    roi = reshape(roi, [], 1);
    % validIdx = xyzPoints(:, 3) < 1.5*maxZ & xyzPoints(:, 3) > 0.5*minZ & roi;
    validIdx = logical(roi) & xyzPoints(:, 3) < maxZ * 1.2;
    xyzPoints = xyzPoints(validIdx, :);
    xyzPointsInImage1 = (camMatrix1 * [xyzPoints, ones(size(xyzPoints, 1), 1)]')';
    xyzPointsInImage1 = floor(xyzPointsInImage1(:, 2:-1:1) ./ xyzPointsInImage1(:, 3));
    pixelColors = impixel(image1Rect, xyzPointsInImage1(: , 1), xyzPointsInImage1(: , 2));


    % create a pointCloud object and assing the color of the original image at the corresponding pixel
    size(xyzPoints)
    % pointCloudDense = pccat([pointCloudDense, pointCloud(xyzPoints, Color=pixelColors)]);
    pointCloudDense = pccat([pointCloudDense, pointCloud(xyzPoints, Color=[0.8, 0.8, 0.8])]);
end

if log
    fprintf('\nDense reconstruction finished after %.2f seconds.\n', toc);
end

end
