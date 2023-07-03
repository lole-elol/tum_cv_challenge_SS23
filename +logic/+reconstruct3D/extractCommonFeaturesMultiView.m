function [matchedPoints1, matchedPoints2, currPoints, currFeatures, indexPairs] = extractCommonFeaturesMultiView(image, prevFeatures, prevPoints, ~, varargin)
    % Extracts common features from two images and returns 
    % the essential matrix and the inlier points.

    p = inputParser;
    p.addOptional('numOctaves', 20);
    p.addOptional('roiBorder', 20);
    p.parse(varargin{:});
    numOctaves = p.Results.numOctaves;
    roiBorder = p.Results.roiBorder;

    % Detect features in the images 
    % TODO: add a parameter to try different feature extraction methods 
    % TODO: consider changing detection algorithm and getting different point types
    % https://de.mathworks.com/help/vision/ug/point-feature-types.html
    roi = [roiBorder, roiBorder, size(image, 2) - 2 * roiBorder, size(image, 1) - 2 * roiBorder];
    currPoints = detectSURFFeatures((image), NumOctaves=numOctaves, ROI=roi);
    currFeatures = extractFeatures(image, currPoints);    
    indexPairs   = matchFeatures(prevFeatures, currFeatures);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
end