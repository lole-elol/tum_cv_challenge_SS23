function [worldPoints, camPoses, tracks] = getTriangulatedPointsMultiView(tracks, camPoses, cameraParams, varargin)
% GETTRIANGLATEDPOINTSMULTIVIEW Get the 3D points from the matched points and the relative pose of thr two cameras
%                               Compute the 3D points from the camera pose
% Inputs:
%   tracks: Matched points between two images
%   camPoses: RCamera poses of all the cameras
%   cameraParams: Camera parameters
%   maxReprojectionError = 5: Maximum reprojection error of the triangulated points
% Outputs:
%   worldPoints: 3D points
%   camPoses: Refined camera poses
%   tracks: Refined matched points


    p = inputParser;
    p.addOptional('maxReprojectionError', 5);
    p.parse(varargin{:});

    maxReprojectionError = p.Results.maxReprojectionError;

    [worldPoints, reprojectionErrors, validIndex] = triangulateMultiview(tracks, camPoses, cameraParams);
    % Refine the 3-D world points and camera poses.
    [worldPoints, camPoses] = bundleAdjustment(worldPoints, tracks, camPoses, cameraParams);

    % Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
    % remove points with high reprojection error
    validIndex = validIndex & (reprojectionErrors < maxReprojectionError);
    worldPoints = worldPoints(validIndex, :);
    tracks = tracks(validIndex);
end