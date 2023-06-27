function [matchedPoints1, matchedPoints2] = extractCommonFeatures(image1, image2, cameraParams, varargin)
% EXTRACTCOMMONFEATURES Extracts common features from two images
% Inputs:
%   image1, image2: images to extract features from
%   cameraParams: camera parameters
%   minQuality = 0.1: minimum quality of the extracted features
%   roiBorder = 200: border of the region of interest
% Outputs:
%   matchedPoints1, matchedPoints2: matched points in the images

p = inputParser;
p.addOptional('minQuality', 0.1);
p.addOptional('roiBorder', 200);
p.parse(varargin{:});
minQuality = p.Results.minQuality;
roiBorder = p.Results.roiBorder;

% Detect features in the images 
% TODO: add a parameter to try different feature extraction methods 
% TODO: consider changing detection algorithm and getting different point types
% https://de.mathworks.com/help/vision/ug/point-feature-types.html
roi = [roiBorder, roiBorder, size(image1, 2) - 2 * roiBorder, size(image1, 1) - 2 * roiBorder];
points1 = detectMinEigenFeatures(image1, MinQuality=minQuality);
points2 = detectMinEigenFeatures(image2, MinQuality=minQuality);

% Extract features from the images
[features1, validPoints1] = extractFeatures(image1, points1);
[features2, validPoints2] = extractFeatures(image2, points2);
% Match the features between the images
indexPairs = matchFeatures(features1, features2);

% Retrieve the locations of the corresponding points for each image
matchedPoints1 = validPoints1(indexPairs(:, 1));
matchedPoints2 = validPoints2(indexPairs(:, 2));

end