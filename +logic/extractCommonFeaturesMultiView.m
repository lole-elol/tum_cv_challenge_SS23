function [matchedPoints1, matchedPoints2, currPoints, currFeatures, indexPairs] = extractCommonFeaturesMultiView(image_1, prevFeatures, prevPoints, ~, varargin)
    % Extracts common features from two images and returns 
    % the essential matrix and the inlier points.

    p = inputParser;
    p.addOptional('numOctaves', 15);
    p.addOptional('roi_border', 20);
    p.parse(varargin{:});
    numOctaves = p.Results.numOctaves;
    roi_border = p.Results.roi_border;

    % Detect features in the images 
    % TODO: add a parameter to try different feature extraction methods 
    % TODO: consider changing detection algorithm and getting different point types
    % https://de.mathworks.com/help/vision/ug/point-feature-types.html
    %roi = [roi_border, roi_border, size(image_1, 2) - 2 * roi_border, size(image_1, 1) - 2 * roi_border];
    currPoints = detectSURFFeatures(image_1);%, NumOctaves=numOctaves, ROI=roi);
    currFeatures = extractFeatures(image_1, currPoints);    
    indexPairs   = matchFeatures(prevFeatures, currFeatures);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
end