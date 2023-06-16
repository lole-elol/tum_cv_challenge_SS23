function [E, rel_pose, status] = getEpipolarGeometry(matched_points1, matched_points2, camera_params, varargin)
    % Get the epipolar geometry between two images. If the estimation fails, relpose is set to identity.
    % Input:
    %   matched_points1, matched_points2: matched points between two images
    %   camera_params: camera parameters
    %   e_max_distance: max distance for inliers
    %   e_confidence: confidence for RANSAC
    %   e_max_num_trials: max number of trials for RANSAC
    % Output:
    %   E: essential matrix
    %   rel_pose: relative pose between two images
    %   status: 0 if success, else if failed
    p = inputParser;
    p.addOptional('e_max_distance', 1.5);
    p.addOptional('e_confidence', 99.99);
    p.addOptional('e_max_num_trials', 1000);
    p.parse(varargin{:});
    e_max_distance = p.Results.e_max_distance;
    e_confidence = p.Results.e_confidence;
    e_max_num_trials = p.Results.e_max_num_trials;

    [E, inliers, status] = estimateEssentialMatrix(matched_points1, matched_points2, camera_params, ...
    MaxDistance=e_max_distance, Confidence=e_confidence, MaxNumTrials=e_max_num_trials);
    if status ~= 0
        rel_pose = rigidtform3d;
    else
        inlier_points1 = matched_points1(inliers);
        inlier_points2 = matched_points2(inliers);
        rel_pose = estrelpose(E, camera_params.Intrinsics, camera_params.Intrinsics, inlier_points1, inlier_points2);    
    end
end