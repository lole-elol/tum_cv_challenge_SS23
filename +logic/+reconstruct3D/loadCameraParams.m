%% Function able to load the params of a text file cameras.txt of the form
% # Camera list with one line of data per camera:
% #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
% # Number of cameras: 1
% 0 PINHOLE 6211 4137 3410.34 3409.98 3121.33 2067.07

function [cameraParams] = loadCameraParams(filename)
% LOADCAMERAPARAMS Load the camera parameters from a text file
% Input:
%   filename - The name of the file to load the camera parameters from
% Output:
%   cameraParams - The camera parameters
file = fopen(filename,'r');
formatSpec = '%d %s %d %d %f %f %f %f';

% Read the camera information from the file
header = textscan(file, '%s', 3, Delimiter='\n');
data = textscan(file, formatSpec, Delimiter=' ');

% Close the file
fclose(file);

% Extract the camera information from the read data
cameraId = data{1};
model = data{2};
width = data{3};
height = data{4};
parameters = [data{5}, data{6}, data{7}, data{8}];

% Display the camera information
% fprintf('Camera ID: %d\n', cameraId);
% % fprintf('Model: %s\n', model);
% fprintf('Width: %d\n', width);
% fprintf('Height: %d\n', height);
% fprintf('Parameters: %.2f %.2f %.2f %.2f\n', parameters);

k = [parameters(1), 0, parameters(3); 0, parameters(2), parameters(4); 0, 0, 1];
% Make width and height double
width = double(width);
height = double(height);
cameraParams = cameraParameters(K=k,ImageSize=[height,width]);
end