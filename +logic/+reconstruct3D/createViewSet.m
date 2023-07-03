function [vSet, prevFeatures, prevPoints] = createViewSet(image1, varargin)
% CREATEVIEWSET Create a viewSet object containing the first view.
% Inputs:
%   image1 - first image
%   numOctaves - number of octaves for SURF feature detection
%   roiBorder - border around the image to exclude from feature detection
% Outputs:
%   vSet - viewSet object containing the first view
%   prevFeatures - SURF features of the first image
%   prevPoints - SURF points of the first image

p = inputParser;
p.addOptional('numOctaves', 20);
p.addOptional('roiBorder', 20);
p.parse(varargin{:});
numOctaves = p.Results.numOctaves;
roiBorder = p.Results.roiBorder;

roi = [roiBorder, roiBorder, size(image1, 2)- 2*roiBorder, size(image1, 1)- 2*roiBorder];
prevPoints = detectSURFFeatures(image1, NumOctaves=numOctaves, ROI=roi);
prevFeatures = extractFeatures(image1, prevPoints);

% Create an empty imageviewset object to manage the data associated with each view.
vSet = imageviewset;

% Add the first view. Place the camera associated with the first view and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, rigidtform3d, Points=prevPoints);
end
