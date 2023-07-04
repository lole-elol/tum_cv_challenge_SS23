function imageOut = preprocessImage(image)
% PREPROCESSIMAGE Perform preprocessing on the images to prepare them for
% feature extraction and matching
% Inputs:
%   image - the image to be preprocessed
% Outputs:
%   imageOut - the preprocessed image

imageGray = rgb2gray(image);

% Remove salt and pepper noise
imageDenoised = medfilt2(imageGray);

% Enhance the contrast
imageEnhanced = adapthisteq(imageDenoised);

% Sharpen the image
imageSharpened = imsharpen(imageEnhanced);

% Remove salt and pepper noise again
imageSharpened = medfilt2(imageSharpened);

imageOut = imageSharpened;
end
