function [points, features] = extractFeatures(image, ~, varargin)
% EXTRACTFEATURES Extracts features from the image
%   [points, features] = EXTRACTFEATURES(image, prevFeatures, method, numOctaves, roiBorder)
%   extracts features from the image using the specified method. 
%   Inputs:
%      image           - The image from which the features should be extracted
%      prevFeatures    - The features from the previous image
%      method = 'SURF' - The method used to extract the features. Can be 'SURF',
%                       'FAST', 'Harris' or 'BRISK'
%      numOctaves = 20 - The number of octaves used for SURF
%      roiBorder = 20  - The border around the image that is not used for SURF
%      minQuality = 0.001 - The minimum quality for minEigen
%   Outputs:
%      points          - The points of the features
%      features        - The features

p = inputParser;
p.addOptional('method', 'SURF', @(x) any(validatestring(x, {'SURF', 'FAST', 'Harris', 'BRISK', 'minEigen'})));
p.addOptional('numOctaves', 20);
p.addOptional('roiBorder', 20);
p.addOptional('minQuality', 0.001);
p.parse(varargin{:});
method = p.Results.method;
numOctaves = p.Results.numOctaves;
roiBorder = p.Results.roiBorder;
minQuality = p.Results.minQuality;

% Detect features in the images 
% https://de.mathworks.com/help/vision/ug/point-feature-types.html
if strcmp(p.Results.method, 'SURF')
    roi = [roiBorder, roiBorder, size(image, 2) - 2 * roiBorder, size(image, 1) - 2 * roiBorder];
    points = detectSURFFeatures((image), NumOctaves=numOctaves, ROI=roi);
    features = extractFeatures(image, points);
elseif strcmp(p.Results.method, 'FAST')
    points = detectFASTFeatures(image);
    features = extractFeatures(image, points);
elseif strcmp(p.Results.method, 'Harris')
    points = detectHarrisFeatures(image);
    features = extractFeatures(image, points);
elseif strcmp(p.Results.method, 'BRISK')
    points = detectBRISKFeatures(image);
    features = extractFeatures(image, points);
elseif strcmp(p.Results.method, 'minEigen')
    points = detectMinEigenFeatures(image, MinQuality=minQuality);
    features = extractFeatures(image, points);
else
    error('Unknown method')
end


end