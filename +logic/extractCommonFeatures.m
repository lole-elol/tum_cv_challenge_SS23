function [matched_points1, matched_points2] = extractCommonFeatures(image_1, image_2, camera_params, varargin)
    % EXTRACTCOMMONFEATURES Extracts common features from two images
    % Inputs:
    %   image_1, image_2: images to extract features from
    %   camera_params: camera parameters
    %   min_quality = 0.1: minimum quality of the extracted features
    %   roi_border = 200: border of the region of interest
    % Outputs:
    %   matched_points1, matched_points2: matched points in the images

    p = inputParser;
    p.addOptional('min_quality', 0.1);
    p.addOptional('roi_border', 200);
    p.parse(varargin{:});
    min_quality = p.Results.min_quality;
    roi_border = p.Results.roi_border;

    % Detect features in the images 
    % TODO: add a parameter to try different feature extraction methods 
    % TODO: consider changing detection algorithm and getting different point types
    % https://de.mathworks.com/help/vision/ug/point-feature-types.html
    roi = [roi_border, roi_border, size(image_1, 2) - 2 * roi_border, size(image_1, 1) - 2 * roi_border];
    points1 = detectMinEigenFeatures(image_1, MinQuality=min_quality);
    points2 = detectMinEigenFeatures(image_2, MinQuality=min_quality);

    % Extract features from the images
    [features1, valid_points1] = extractFeatures(image_1, points1);
    [features2, valid_points2] = extractFeatures(image_2, points2);
    % Match the features between the images
    index_pairs = matchFeatures(features1, features2);

    % Retrieve the locations of the corresponding points for each image
    matched_points1 = valid_points1(index_pairs(:, 1));
    matched_points2 = valid_points2(index_pairs(:, 2));

end