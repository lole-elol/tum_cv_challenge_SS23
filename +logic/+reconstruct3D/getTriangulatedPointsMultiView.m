function [worldPoints, camPoses, tracks] = getTriangulatedPointsMultiView(tracks, camPoses, camera_params, varargin)
    % Get the 3D points from the matched points and the relative pose of thr two cameras
    % Compute the 3D points from the camera pose
    p = inputParser;
    p.addOptional('image', []);
    p.addOptional('maxReprojectionError', 5);
    p.parse(varargin{:});
    maxReprojectionError = p.Results.maxReprojectionError;

    [worldPoints, reprojectionErrors, valid_index] = triangulateMultiview(tracks, camPoses, camera_params);
    % Refine the 3-D world points and camera poses.
    [worldPoints, camPoses] = bundleAdjustment(worldPoints, tracks, camPoses, camera_params);

    % Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
    % remove points with high reprojection error
    valid_index = valid_index & (reprojectionErrors < maxReprojectionError);
    worldPoints = worldPoints(valid_index, :);
    tracks = tracks(valid_index);
end