function [imageOut, imageGray, imageCanny] = preprocessImage(image, cameraParams, varargin)
% PREPROCESSIMAGE Perform preprocessing on the images to prepare them for
% feature extraction and matching
% Inputs:
%   image - the image to be preprocessed
%   cameraParams - the camera parameters for the camera that took the image
%   cannyGaussianFilterSize - the size of the gaussian filter to apply to the image before canny edge detection
%   cannyThreshold - the threshold values to use for the canny edge detector
%   gaussCannyCombineFilterSize - the size of the gaussian filter to apply to the image after combining the canny edges and the original image
% Outputs:
%   imageOut - the preprocessed image
%   imageGray - the grayscale version of the image
%   imageCanny - the image after applying the canny edge detector

p = inputParser;

p.addOptional('cannyGaussianFilterSize', 5);
p.addOptional('cannyThreshold', [0.1 0.2]);
p.addOptional('gaussCannyCombineFilterSize', 3);
p.parse(varargin{:});

cannyGaussianFilterSize = p.Results.cannyGaussianFilterSize;
cannyThreshold = p.Results.cannyThreshold;
gaussCannyCombineFilterSize = p.Results.gaussCannyCombineFilterSize;

% Remove lens distortion from the images
% TODO: We need to crop (maximum rectangle) the timages to only conatin valid data becasue undinstroting them results in black borders
% which conflict with the later steps
%I1_gray = undistortImage(I1_gray, cameraParams);
%I2_gray = undistortImage(I2_gray, cameraParams);

% Apply a canny detector to the images to get the edges of the objects in the images
% pre filter image with gaussian filter to remove noise and improve canny results
% Merge canny edges and original image with a gaussian filter
imageGray = rgb2gray(image);
imageBlured = imgaussfilt(imageGray, cannyGaussianFilterSize);
imageCanny = edge(imageBlured, 'Canny', cannyThreshold);
% TODO: Dont simply add them but maybe make first gray image darker.
imageOut = imgaussfilt(imageCanny + double(imageGray)/255, gaussCannyCombineFilterSize); 
end
