imageDir = "test/old_computer/images";
imageFiles = dir(fullfile(imageDir, '*.jpg')); % Assuming JPEG format for the images

numImages = numel(imageFiles);
images = cell(1, numImages);

for i = 1:numImages
    imagePath = fullfile(imageDir, imageFiles(i).name);
    images{i} = imread(imagePath);
end

% Load the camera parameters
cameraParams = logic.reconstruct3D.loadCameraParams('test/old_computer/cameras.txt');

% Reconstruct3d multiview


