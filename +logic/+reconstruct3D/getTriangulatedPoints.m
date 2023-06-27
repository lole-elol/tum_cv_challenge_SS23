function [pointCloudInstance] = getTriangulatedPoints(matchedPoints1, matchedPoints2, cameraParams, relPose, varargin)
% GETTRIANGULATEDPOINTS Compute the 3D points from the camera pose and the matched points
% Input:
%   matchedPoints1: matched points in image 1
%   matchedPoints2: matched points in image 2
%   cameraParams: camera parameters
%   relPose: relative pose between the two cameras
%   image = []: image to get the color of the points
%   maxReprojectionError = 5: maximum reprojection error to remove outliers
% Output:
%   pointCloudInstance: point cloud of the 3D points

p = inputParser;
p.addOptional('image', []);
p.addOptional('maxReprojectionError', 5);
p.parse(varargin{:});
image = p.Results.image;
maxReprojectionError = p.Results.maxReprojectionError;

camMatrix1 = cameraProjection(cameraParams.Intrinsics, rigidtform3d);
camMatrix2 = cameraProjection(cameraParams.Intrinsics, pose2extr(relPose));
[worldPoints, reprojectionError, validIndex] = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);
% TODO: do bundle adjustment

% Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
% remove points with high reprojection error
validIndex = validIndex & (worldPoints(:, 3) > 0) & (reprojectionError < maxReprojectionError);
worldPoints = worldPoints(validIndex, :);

% Get the color of each reconstructed point
% if no image is passed, return color as the depth
if isempty(image)
    maxZ = max(worldPoints(:, 3));
    color = [worldPoints(:, 3) / maxZ, zeros(size(worldPoints, 1), 1), zeros(size(worldPoints, 1), 1)];
    pointCloudInstance = pointCloud(worldPoints, Color=color);
else
    points = matchedPoints1.Location;
    numPixels = size(image, 1) * size(image, 2);
    allColors = reshape(image, [numPixels, 3]);
    colorIdx = sub2ind(size(image), round(points(validIndex, 2)), round(points(validIndex, 1)));
    color = allColors(colorIdx, :);
    pointCloudInstance = pointCloud(worldPoints, Color=color);
end
end