function image_out = preprocessing(image, camera_params, varargin)
% This function performs preprocessing on the images to prepare them for
% featur extraction and matching

p = inputParser;

p.addOptional('canny_gaussian_filter_size', 5);
p.addOptional('canny_threshold', [0.1 0.2]);
p.addOptional('gauss_canny_combine_filter_size', 3);
p.parse(varargin{:});

canny_gaussian_filter_size = p.Results.canny_gaussian_filter_size;
canny_threshold = p.Results.canny_threshold;
gauss_canny_combine_filter_size = p.Results.gauss_canny_combine_filter_size;

% Remove lens distortion from the images
% TODO: We need to crop (maximum rectangle) the timages to only conatin valid data becasue undinstroting them results in black borders
% which conflict with the later steps
%I1_gray = undistortImage(I1_gray, camera_params);
%I2_gray = undistortImage(I2_gray, camera_params);


% Apply a canny detector to the images to get the edges of the objects in the images
% pre filter image with gaussian filter to remove noise and improve canny results
% Merge canny edges and original image with a gaussian filter


image_gray = rgb2gray(image);
image_blured = imgaussfilt(image_gray, canny_gaussian_filter_size);
image_canny = edge(image_blured, 'Canny', canny_threshold);
image_out = imgaussfilt(image_canny + double(image_gray)/255, gauss_canny_combine_filter_size);

% TODO: Take a look at Hough transformation (i.e Fourier transform) to get lines in the images


end