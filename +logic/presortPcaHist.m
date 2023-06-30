function [images, index, features] = presortPcaHist(inPath)
% PRESORTPCAHIST Sort images via the PCA of their histograms.
%  Project the feature space to 1D and sort the images according to the
%
% Inputs:
%   inPath path to the folder containing the images
% Outputs:
%   images: cell array of images
%   index: sorted index of images
%   features: feature vector of images

% inPath = "test/delivery_area_dslr_undistorted/images";

imagefiles = dir(append(inPath, '/*.jpg'));

nfiles = length(imagefiles);    % Number of files found

images = cell(1, nfiles);
for ii=1:nfiles
    currentfilename = append(inPath, '/', imagefiles(ii).name);
    currentimage = rgb2gray(imresize(imread(currentfilename),0.5));
    images{ii} = currentimage;
end

% histograms of images in the dataset
image_hist = cellfun(@(x) imhist(x), images, 'UniformOutput', false);
% normalize histograms
image_hist_norm = cellfun(@(x) x/max(x), image_hist, 'UniformOutput', false);

X = cell2mat(image_hist_norm)';
[coeff,score,latent,tsquared,explained,mu] = pca(X );
Y = X*coeff(:, 1);
[value, index] = sort(Y);

features = Y;


end
% similarity = zeros(nfiles, nfiles);
% % compute similarity matrix via pdist2
% % using ecuclidean distance
% for i = 1:nfiles
%     for j = 1:nfiles
%         similarity(i,j) = pdist2(image_hist_norm{i}', image_hist_norm{j}');
%     end
% end



