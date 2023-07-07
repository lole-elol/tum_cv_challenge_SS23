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

switch featureType
  case "FFT2"
    fft_img = cellfun(@(img) fftshift(fft2(img)), images, 'UniformOutput', false);
    feature_images = cellfun(@(img) log(1 + abs(img)), fft_img, 'UniformOutput', false);
  case "HIST"
    feature_images = cellfun(@(img) imhist(img), images, 'UniformOutput', false);
  case "PCA"
    feature_images = images;
  otherwise
    error("Invalid feature type: %s", featureType);
end

% Create row vectors
feature_images = cellfun(@(img) img(:), feature_images, 'UniformOutput', false);

if normalize
  feature_images = cellfun(@(img) img / max(img), feature_images, 'UniformOutput', false);
end

% Calculate features with PCA
X = cell2mat(feature_images)';
[coeff, ~, ~, ~, ~, ~] = pca(X);
features = (X * coeff(:, 1:featureLength))';

if sortNearestNeighbors
  % Sort by nearest neighbors
  index = logic.sortNearestNeighbour(features);
else
  % Sort by 1D features
  [~, index] = sort(features(1, :));
end

% Sort images by index
images = images(index);

end