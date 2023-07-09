function [features, similarity] = similarity(images, varargin)
%SIMILARITY Compute similarity between images
%
% Inputs:
%   images: cell array of images (Must have the dimensions 1xN)
%   featureLength = 1: length of feature vector
%   featureType = 'HIST': Type of similarity algorithm to use. Options are: FFT2, HIST, PCA
%   normalize = true: Wether to normalize the pca analyzed matrix
%   lazy = false: Wether to wheight the similarity matrix based on the distance between images

% Outputs:
%   features: feature matrix
%   similarity: similarity matrix

% Parse input
p = inputParser;
p.addOptional('featureLength', 1, @isnumeric);
p.addOptional('featureType', 'HIST', @(x) isstring(x) || ischar(x));
p.addOptional('normalize', true, @islogical);
p.addOptional('lazy', true, @islogical);
p.parse(varargin{:});

featureLength = p.Results.featureLength;
featureType = p.Results.featureType;
normalize = p.Results.normalize;
lazy = p.Results.lazy;

% Compute features
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

% Calculate similarity matrix
similarity = pdist2(features', features');

% Lazy similarity
if lazy
  n = length(images);
  win = gausswin(2*n);

  weights = win(1:n);
  weights(1) = weights(1)/2;
  %wheights = wheights / sum(wheights);

  weightMat = toeplitz(weights, [weights(1), zeros(1, n-1)]);
  weightMat = weightMat + weightMat';

  similarity = similarity .* weightMat;
end

end
