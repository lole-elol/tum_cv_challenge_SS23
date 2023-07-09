function images = loadImages(imageDir, varargin)
% LOADIMAGES Load the images from the specified directory
% Inputs:
%   imageDir - The directory containing the images
%   log = true - Whether to log the progress to the console
%   numImages = inf - The number of images to load
% Outputs:
%   images - A cell array containing the loaded images

% Parse the input arguments
p = inputParser;
addParameter(p, 'log', true, @islogical);
addParameter(p, 'numImages', inf, @isnumeric);
parse(p, varargin{:});
log = p.Results.log;
numImages = p.Results.numImages;

imageFiles = dir(fullfile(imageDir, '*.JPG')); % Assuming JPEG format for the images TODO: add support for other formats

numImages = min(numImages, length(imageFiles));  % Either load all the images or the specified number of images
images = cell(1, numImages);

% Load the images
if log
    fprintf("Loading images from %s\n", imageDir);
end

for i = 1:numImages
    if log
        fprintf('Loading image %d of %d: %s\r', i, numImages, imageFiles(i).name);
    end
    imagePath = fullfile(imageDir, imageFiles(i).name);
    images{i} = imread(imagePath);
end

end