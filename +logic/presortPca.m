function [images, index, features] = presortPca(images, varargin)
% PRESORTPCA  Sorts images by PCA via projection to a 1D space. - DEPRECATED
%
% Inputs:
%   images:  Cell array of images
%   featureLength = 1: Length of feature vector
%
% Outputs:
%   images:  Sorted images
%   index:   Index of sorted images
%   features: features of images

% inPath = "test/delivery_area_dslr_undistorted/images";

p = inputParser;
p.addOptional('featureLength', 1);

p.parse(varargin{:});
featureLength = p.Results.featureLength;

%imagefiles = dir(append(inPath, '/*.jpg'));
%
%nfiles = length(imagefiles);    % Number of files found
%
%images = cell(1, nfiles);
%for ii=1:nfiles
%    currentfilename = append(inPath, '/', imagefiles(ii).name);
%    currentimage = rgb2gray(imresize(imread(currentfilename),0.07));
%    images{ii} = currentimage;
%end

% stack images into cloumn vector
images_gaussed = cellfun(@(x) imgaussfilt(x, 10), images, 'UniformOutput', false);
images_flat = cellfun(@(x) x(:), images_gaussed, 'UniformOutput', false);
% concat column vectors into matrix
X = im2double(cell2mat(images_flat)');

[coeff,score,latent,tsquared,explained,mu] = pca(X );
Y = X*coeff(:, featureLength);
[value, index] = sort(Y(:, 1));

features = Y;

end

% figure
% imshow(images{index(1)});
% disp(value(1));
% figure
% imshow(images{index(2)});
% disp(value(2));
% figure
% imshow(images{index(3)});
% disp(value(3));
% figure
% imshow(images{index(4)});
% disp(value(4));


% figure
% imshow(images{index(15)});
% disp(value(15));
% figure
% imshow(images{index(16)});
% disp(value(16));
% figure
% imshow(images{index(17)});
% disp(value(17));
% figure
% imshow(images{index(18)});
% disp(value(18));
% figure
% imshow(images{index(19)});
% disp(value(19));