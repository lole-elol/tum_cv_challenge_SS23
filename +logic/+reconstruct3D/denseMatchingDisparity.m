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
addOptional(p, 'scalingFactor', 0.1, @isnumeric);
addOptional(p, 'numImages', Inf, @isnumeric);
parse(p, pointCloudInstance, images, camPoses, cameraParams, varargin{:});
log = p.Results.log;
scalingFactor = p.Results.scalingFactor;
numImages = p.Results.numImages;

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
maxZ = pointCloudInstance.ZLimits(2);
minZ = pointCloudInstance.ZLimits(1);
maxY = pointCloudInstance.YLimits(2);
minY = pointCloudInstance.YLimits(1);
maxX = pointCloudInstance.XLimits(2);
minX = pointCloudInstance.XLimits(1);


% Create a dense point cloud from the triangulated points and camera poses.
for i = 1:(numImages-1)
    if log
        fprintf('Dense reconstruction of image %d of %d \r', i, numImages-1);
    end
    image1 = images{i};
    image2 = images{i+1};
    A1 = camPoses(i, :).AbsolutePose.A;
    A2 = camPoses(i+1, :).AbsolutePose.A;

    % figure
    % imshowpair(image1, image2, 'montage');

    % Compute stereo parameters and rectify the stereo images.
    relPose = rigidtform3d(A2);
    stereoParams = stereoParameters(cameraParams, cameraParams, relPose);
    [image1Rect, image2Rect, ~, camMatrix1] = rectifyStereoImages(image1, image2, stereoParams);
    % figure
    % imshowpair(image1Rect, image2Rect, 'montage');
    A = stereoAnaglyph(image1Rect, image2Rect);
    figure
    imshow(A)
    title("Red-Cyan composite view of the rectified stereo pair image")
    % Compute disparity.
    disparityRange = [0 128];
    disparityMap = disparitySGM(rgb2gray(image1Rect), rgb2gray(image2Rect), DisparityRange=disparityRange, UniquenessThreshold=0);
    figure
    imshow(disparityMap, disparityRange);
    % colormap jet
    % colorbar


    % Reconstruct the 3-D world coordinates of points corresponding to each pixel from the disparity map.
    xyzPoints = reconstructScene(disparityMap, stereoParams);
    size(xyzPoints)
    % Filter points that are too far away and transform the MxNx3 matrix into a Nx3 matrix
    % roiBorder = 20;  % TODO: make this a parameter
    % roi = zeros(size(disparityMap));
    % roi(roiBorder:end-roiBorder, roiBorder:end-roiBorder) = 1;
    % roi = reshape(roi, [], 1);

    xyzPoints = reshape(xyzPoints, [], 3);
    validIdx = xyzPoints(:, 3) < 1.5*maxZ & xyzPoints(:, 3) > 0.5*minZ ...
        & xyzPoints(:, 2) < 1.5*maxY & xyzPoints(:, 2) > 0.5*minY ...
        & xyzPoints(:, 1) < 1.5*maxX & xyzPoints(:, 1) > 0.5*minX;
    xyzPoints = xyzPoints(validIdx, :);

    size(xyzPoints)

    % Transform the 3D points to the coordinate system of the first image.
    % To get a color for each point, map the 3D point to the corresponding pixel in the first image.
    xyzPointsInImage1 = (camMatrix1 * [xyzPoints, ones(size(xyzPoints, 1), 1)]')';
    xyzPointsInImage1 = floor(xyzPointsInImage1(:, 2:-1:1) ./ xyzPointsInImage1(:, 3));
    pixelColors = impixel(image1Rect, xyzPointsInImage1(: , 2), xyzPointsInImage1(: , 1));
    pixelColors = pixelColors / 255;


    % create a pointCloud object and assing the color of the original image at the corresponding pixel
    pointCloudDense = pccat([pointCloudDense, pointCloud(xyzPoints, Color=pixelColors)]);
end
if log
    fprintf('\nDense reconstruction finished after %.2f seconds.\n', toc);
end

end
