function [E, rel_pose, inliers] = getEpipolarGeometry(matchedPoints1, matchedPoints2, camera_params, varargin)
    % Get the epipolar geometry between two images. If the estimation fails, relpose is set to identity.
    % Input:
    %   matched_points1, matched_points2: matched points between two images
    %   camera_params: camera parameters
    %   e_max_distance = 1.5: max distance for RANSAC
    %   e_confidence = 99.99: confidence for RANSAC
    %   e_max_num_trials = 1000: max number of trials for RANSAC
    % Output:
    %   E: essential matrix
    %   rel_pose: relative pose between two images
    %   status: 0 if success, else if failed
    p = inputParser;
    p.addOptional('e_max_distance', 5);
    p.addOptional('e_confidence', 99.5);
    p.addOptional('e_max_num_trials', 100000);
    p.addOptional('e_valid_point_fraction', 0.8);
    p.parse(varargin{:});
    e_max_distance = p.Results.e_max_distance;
    e_confidence = p.Results.e_confidence;
    e_max_num_trials = p.Results.e_max_num_trials;
    e_valid_point_fraction = p.Results.e_valid_point_fraction;

    if ~isnumeric(matchedPoints1)
        matchedPoints1 = matchedPoints1.Location;
    end
    
    if ~isnumeric(matchedPoints2)
        matchedPoints2 = matchedPoints2.Location;
    end

    [E, inliers] = estimateEssentialMatrix(matchedPoints1, matchedPoints2, camera_params, ...
    'MaxDistance', e_max_distance, 'Confidence', e_confidence, 'MaxNumTrials', e_max_num_trials);
    inlierPoints1 = matchedPoints1(inliers, :);
    inlierPoints2 = matchedPoints2(inliers, :);
    [rel_pose, validPointFraction] = estrelpose(E, camera_params.Intrinsics, inlierPoints1, inlierPoints2);
    % validPointFraction is the fraction of inlier points that project in
    % front of both cameras. If the this fraction is too small, then the
    % fundamental matrix is likely to be incorrect.
     if validPointFraction > e_valid_point_fraction % TODO: check if this is a good threshold
        return;
     end
end