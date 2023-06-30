function [pointCloudInstance, camPoses] = getTriangulatedPointsMultiView(tracks, camPoses, camera_params, varargin)
    % Get the 3D points from the matched points and the relative pose of thr two cameras
    % Compute the 3D points from the camera pose
    p = inputParser;
    p.addOptional('image', []);
    p.addOptional('max_reprojection_error', 20);
    p.parse(varargin{:});
    max_reprojection_error = p.Results.max_reprojection_error;

    [worldPoints, reprojectionErrors, valid_index] = triangulateMultiview(tracks, camPoses, camera_params);
    % TODO: do bundle adjustment
    % Refine the 3-D world points and camera poses.
    [worldPoints, camPoses] = bundleAdjustment(worldPoints, tracks, camPoses, camera_params);

    % Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
    % remove points with high reprojection error
    valid_index = valid_index & (worldPoints(:, 3) > 0) & (reprojectionErrors < max_reprojection_error);
    worldPoints = worldPoints(valid_index, :);
    maxZ = max(worldPoints(:, 3));
    color = repmat([0.5, 0.5, 0.5], size(worldPoints, 1), 1);
    pointCloudInstance = pointCloud(worldPoints, Color=color);
end