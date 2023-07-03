function [pointCloudInstance, camPoses] = getTriangulatedPointsMultiView(tracks, camPoses, camera_params, varargin)
    % Get the 3D points from the matched points and the relative pose of thr two cameras
    % Compute the 3D points from the camera pose
    p = inputParser;
    p.addOptional('image', []);
    p.addOptional('maxReprojectionError', 20);
    p.parse(varargin{:});
    maxReprojectionError = p.Results.maxReprojectionError;

    [worldPoints, reprojectionErrors, valid_index] = triangulateMultiview(tracks, camPoses, camera_params);
    % Refine the 3-D world points and camera poses.
    [worldPoints, camPoses] = bundleAdjustment(worldPoints, tracks, camPoses, camera_params);

    % Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
    % remove points with high reprojection error
    valid_index = valid_index & (reprojectionErrors < maxReprojectionError);
    worldPoints = worldPoints(valid_index, :);
    
    % TODO: color the points based on the image
    maxZ = max(worldPoints(:, 3));
    color = repmat([0.5, 0.5, 0.5], size(worldPoints, 1), 1);
    pointCloudInstance = pointCloud(worldPoints, Color=color);
end