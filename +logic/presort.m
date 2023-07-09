function [images, index] = presort(images, varargin)
% PRESORTIMAGES  Sort by similarity to each other
%
% Inputs:
%   images:  Cell array of images
%   featureLength = 1: Length of feature vector (only used if `sortNearestNeighbors` is true)
%   featureType = 'FFT2': Type of similarity algorithm to use. Options are: FFT2, HIST, PCA
%   sortNearestNeighbors = true: Whether to sort by nearest neighbors
%   normalize = true: Whether to normalize the pca analyzed matrix
%
% Outputs:
%   images:  Sorted images
%   index:   Index of sorted images

% Parse input
p = inputParser;
p.addOptional('featureLength', 1, @isnumeric);
p.addOptional('featureType', 'FFT2', @(x) isstring(x) || ischar(x));
p.addOptional('sortNearestNeighbors', true, @islogical);
p.addOptional('normalize', true, @islogical);
p.parse(varargin{:});

featureLength = p.Results.featureLength;
featureType = p.Results.featureType;
sortNearestNeighbors = p.Results.sortNearestNeighbors;
normalize = p.Results.normalize;

[features, similarity] = logic.similarity(images, featureLength=featureLength, featureType=featureType, normalize=normalize);

if sortNearestNeighbors
  % Sort by nearest neighbors
  index = logic.sortNearestNeighbour(similarity);
else
  % Sort by 1D features
  [~, index] = sort(features(1, :));
end

% Sort images by index
images = images(index);

end