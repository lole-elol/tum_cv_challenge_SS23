function [world_points, camPoses] = getTriangulatedPointsMultiView(tracks, camPoses, camera_params, varargin)
    % Get the 3D points from the matched points and the relative pose of thr two cameras
    % Compute the 3D points from the camera pose
    p = inputParser;
    p.addOptional('image', []);
    p.addOptional('max_reprojection_error', 5);
    p.parse(varargin{:});
    max_reprojection_error = p.Results.max_reprojection_error;

    [world_points, reprojectionErrors, valid_index] = triangulateMultiview(tracks, camPoses, camera_params);
    % TODO: do bundle adjustment
    % Refine the 3-D world points and camera poses.
    [world_points, camPoses] = bundleAdjustment(world_points, tracks, camPoses, camera_params);

    % Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
    % remove points with high reprojection error
    % valid_index = valid_index & (world_points(:, 3) > 0) & (reprojectionErrors < max_reprojection_error);
    % world_points = world_points(valid_index, :);

end