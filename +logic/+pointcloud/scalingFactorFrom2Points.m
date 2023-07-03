function [scalingFactor, worldPoints] = scalingFactorFrom2Points(points, distance, camPose1, camPose2, cameraIntrinsics)
% SCALINGFACTORFROM2POINTS  Calculate the scaling factor from 2 points in
% 2 different perspectives (4 points in total), given the distance between the points in the real
% world.
%
%   scalingFactor = scalingFactorFrom2Points(images, points, distance)
% Inputs:
%   points: a cell array of 2x2 matrices, each matrix contains the
%   coordinates of the 2 points in the image, each row is a point
%   distance: distance between the points in the real world
%   camPose1: camera pose of the first camera
%   camPose2: camera pose of the second camera
%   cameraIntrinsics: camera intrinsics of the cameras
% Outputs:
%   scalingFactor: the scaling factor between the images
%   worldPoints: the 3D coordinates of the 2 points in the real world

camMatrix1 = cameraProjection(cameraIntrinsics, camPose1);
camMatrix2 = cameraProjection(cameraIntrinsics, camPose2);
[worldPoints, reprojectionError, validIndex] = triangulate(points{1}, points{2}, camMatrix1, camMatrix2);

scalingFactor = distance / norm(worldPoints(1,:) - worldPoints(2,:));

end