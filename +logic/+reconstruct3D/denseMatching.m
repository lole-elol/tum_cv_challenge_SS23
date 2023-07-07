function pointCloudDense = denseMatching(pointCloudInstance, imagesOriginal, cameraParams)
% TODO: add description

% === 4. Dense reconstruction ===
if log
    fprintf('Dense reconstruction\n');
end
% Create a dense point cloud from the triangulated points and camera poses.
numImagesDenseRec = 2;
for i = 1:(numImagesDenseRec-1)
    if log
        fprintf('Dense reconstruction of image %d of %d \r', i, numImages-1);
    end
    image1 = imagesOriginal{i};
    image2 = imagesOriginal{i+1};
    % figure
    % imshowpair(image1, image2, 'montage');
    
    % Compute stereo parameters and rectify the stereo images.
    relPose = rigidtform3d(poses(vSet, i).AbsolutePose.A \ poses(vSet, i+1).AbsolutePose.A);
    stereoParams = stereoParameters(cameraParams, cameraParams, relPose);
    %[image1Rect, image2Rect] = rectifyStereoImages(rgb2gray(image1), rgb2gray(image2), stereoParams);
    [image1Rect, image2Rect, ~, camMatrix1] = rectifyStereoImages(image1, image2, stereoParams);
    figure
    imshowpair(image1Rect, image2Rect, 'montage');
    
    % Compute disparity.
    disparityRange = [0 128];
    disparityMap = disparitySGM(rgb2gray(image1Rect), rgb2gray(image2Rect), DisparityRange=disparityRange);
    figure
    imshow(disparityMap, disparityRange);
    colormap jet
    colorbar


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
    pointCloudInstance = pccat([pointCloudInstance, pointCloud(xyzPoints, Color=pixelColors)]);
end
if log
    fprintf('\nDense reconstruction finished after %.2f seconds.\n', toc);
end
end
