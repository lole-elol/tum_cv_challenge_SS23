function [images, index] = presortPcaFFT2(inPath)
% PRESORTPCAFFT2 - Sorts images by PCA on FFT2
%
% Inputs:
%   inPath path to folder with images
% Outputs:
%   images: cell array of images
%   index: sorted index of images

% inPath = "test/delivery_area_dslr_undistorted/images";

imagefiles = dir(append(inPath, '/*.jpg'));

nfiles = length(imagefiles);    % Number of files found

images = cell(1, nfiles);
for ii=1:nfiles
    currentfilename = append(inPath, '/', imagefiles(ii).name);
    currentimage = rgb2gray(imresize(imread(currentfilename),0.07));
    images{ii} = currentimage;
end

fft_images = cellfun(@(x) fft2(x), images, 'UniformOutput', false);
fft_images_shiffted= cellfun(@(x) fftshift(x), fft_images, 'UniformOutput', false);
fft_images_log = cellfun(@(x) log(1+abs(x)), fft_images_shiffted, 'UniformOutput', false);
% flatten fft images
fft_images_flatt = cellfun(@(x) x(:), fft_images_log, 'UniformOutput', false);
%normalize fft images
fft_images_norm = cellfun(@(x) x/max(x), fft_images_flatt, 'UniformOutput', false);

X = cell2mat(fft_images_norm)';
[coeff,score,latent,tsquared,explained,mu] = pca(X );
Y = X*coeff(:, 1);
[value, index] = sort(Y);

end

% F = fft2(gray_img);
% Fsh = fftshift(F);
% log_img = log(1+abs(Fsh));

% end