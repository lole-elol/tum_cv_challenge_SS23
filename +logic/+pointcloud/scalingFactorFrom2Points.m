function [scalingFactor, worldPoints] = scalingFactorFrom2Points(points, camPoses, camParams, distance)
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

tracks = [ pointTrack([1, 2], points{1}),  pointTrack([1, 2], points{2})];
[worldPoints, reprojectionErrors, valid_index] = triangulateMultiview(tracks, camPoses, camParams);
scalingFactor = distance / norm(worldPoints(1,:) - worldPoints(2,:));

end