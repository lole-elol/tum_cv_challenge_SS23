function [image_out, image_gray, image_canny] = preprocessImage(image, camera_params, varargin)
    % PREPROCESSIMAGE Perform preprocessing on the images to prepare them for
    % feature extraction and matching
    % Inputs:
    %   image - the image to be preprocessed
    %   camera_params - the camera parameters for the camera that took the image
    %   canny_gaussian_filter_size - the size of the gaussian filter to apply to the image before canny edge detection
    %   canny_threshold - the threshold values to use for the canny edge detector
    %   gauss_canny_combine_filter_size - the size of the gaussian filter to apply to the image after combining the canny edges and the original image
    % Outputs:
    %   image_out - the preprocessed image
    %   image_gray - the grayscale version of the image
    %   image_canny - the image after applying the canny edge detector
    
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
    % TODO: Dont simply add them but maybe make first gray image darker.
    image_out = imgaussfilt(image_canny + double(image_gray)/255, gauss_canny_combine_filter_size); 

    
    
