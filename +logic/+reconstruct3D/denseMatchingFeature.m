function pointCloudDense = denseMatchingFeature(pointCloudInstance, images, camPoses, vSet, cameraParams, maxZ, varargin)
% DENSEMATCHINGFEATURES 3-D reconstruction of a scene using dense feature matching

numImages = length(images);
scalingFactor = 0.5;

for i = 1:numImages
    images{i} = imresize(images{i}, scalingFactor);
end
I1 = images{1};
cameraParams = logic.reconstruct3D.scaleCameraParameters(cameraParams, scalingFactor, size(im2gray(I1)));


% Detect dense feature points. Use an ROI to exclude points close to the
% image edges.

border = 30;
roi = [border, border, size(I1, 2)- 2*border, size(I1, 1)- 2*border];
imagePoints1 = detectMinEigenFeatures(im2gray(I1), ROI = roi, MinQuality = 0.001);

% Create the point tracker
tracker = vision.PointTracker(MaxBidirectionalError=1, NumPyramidLevels=5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

intrinsics = cameraParams.Intrinsics;

% Track the points
for i = 2:numImages
    I2 = images{i};
    A1 = poses(vSet, i-1).AbsolutePose.A
    A2 = poses(vSet, i).AbsolutePose.A

    relPose = rigidtform3d(A1 \ A2);
    [imagePoints2, validIdx] = step(tracker, I2);
    matchedPoints1 = imagePoints1(validIdx, :);
    matchedPoints2 = imagePoints2(validIdx, :);

    % Compute the camera matrices for each position of the camera
    % The first camera is at the origin looking along the Z-axis. Thus, its
    % transformation is identity.
    camMatrix1 = cameraProjection(intrinsics, rigidtform3d);
    camMatrix2 = cameraProjection(intrinsics, pose2extr(relPose));

    % Compute the 3-D points
    points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

    % Get the color of each reconstructed point
    numPixels = size(I1, 1) * size(I1, 2);
    allColors = reshape(I1, [numPixels, 3]);
    colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matchedPoints1(:,2)), ...
        round(matchedPoints1(:, 1)));
    color = allColors(colorIdx, :);

    % Create the point cloud
    pointCloudDense = pccat([pointCloudInstance, pointCloud(points3D, 'Color', color)]);

end

end