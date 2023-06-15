function [point_cloud] = getTriangulatedPoints(matched_points1, matched_points2, camera_params, rel_pose, varargin)
    % Get the 3D points from the matched points and the relative pose of thr two cameras
    % Compute the 3D points from the camera pose
    p = inputParser;
    p.addOptional('image', []);
    p.addOptional('max_reprojection_error', 5);
    p.parse(varargin{:});
    image = p.Results.image;
    max_reprojection_error = p.Results.max_reprojection_error;

    camMatrix1 = cameraProjection(camera_params.Intrinsics, rigidtform3d);
    camMatrix2 = cameraProjection(camera_params.Intrinsics, pose2extr(rel_pose));
    [world_points, reprojection_error, valid_index] = triangulate(matched_points1, matched_points2, camMatrix1, camMatrix2);
    % TODO: do bundle adjustment

    % Remove points with negative z coordinate and points that are too far away from the camera as they are probably outliers
    % remove points with high reprojection error
    valid_index = valid_index & (world_points(:, 3) > 0) & (reprojection_error < max_reprojection_error);
    world_points = world_points(valid_index, :);

    % Get the color of each reconstructed point
    % if no image is passed, return color as the depth
    if isempty(image)
        max_z = max(world_points(:, 3));
        color = [world_points(:, 3) / max_z, zeros(size(world_points, 1), 1), zeros(size(world_points, 1), 1)];
        point_cloud = pointCloud(world_points, 'Color', color);
    else
        points = matched_points1.Location;
        numPixels = size(image, 1) * size(image, 2);
        allColors = reshape(image, [numPixels, 3]);
        colorIdx = sub2ind(size(image), round(points(valid_index, 2)), round(points(valid_index, 1)));
        color = allColors(colorIdx, :);
        point_cloud = pointCloud(world_points, 'Color', color);
    end
end