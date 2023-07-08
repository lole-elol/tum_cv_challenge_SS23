function pointCloudColored = getColoredPointCloud(worldPoints, tracks, imagesOriginal)
% GETCOLOREDPOINTCLOUD Get a colored point cloud from the world points and
% the tracks. The colors are taken from the original images in the
% corresponding points.
% Inputs:
%   worldPoints - the world points
%   tracks - the tracks
%   imagesOriginal - the original images as a cell array of MxNx3 matrices
% Outputs:
%   pointCloudColored - the colored point cloud

% First get the index of the view in which we can see each point and the
% corresponding image point. Save them in views and imagePoints.
p = inputParser;
addRequired(p, 'worldPoints');
addRequired(p, 'tracks');
addRequired(p, 'imagesOriginal');
parse(p, worldPoints, tracks, imagesOriginal);

% Then, for each image, take the colors of the points in the image and
numImages = length(imagesOriginal);
views = zeros(length(tracks), 1);
imagePoints = zeros(length(tracks), 2);
colorsPointCloud = zeros(length(tracks), 3);

for i = 1:length(tracks)
    allViews = tracks(i).ViewIds;
    allPoints = tracks(i).Points;
    views(i) = allViews(1);
    imagePoints(i, :) = round(allPoints(1, :));
end

% For each image, take the colors of the points in the image and
% save them in colorsPointCloud in the corresponding indices.
for i = 1:numImages
    image = imagesOriginal{i};
    pointsIndicesInImage = find(views == i);
    imagePointsInView = imagePoints(views == i, :);
    % Take only the colors in the diagonal of imageCol
    colorsPointCloud(pointsIndicesInImage, :) = impixel(image, imagePointsInView(:, 1), imagePointsInView(:, 2));
end

% Normalize the colors and create the point cloud with the colors
colorsPointCloud = colorsPointCloud / 255;
pointCloudColored = pointCloud(worldPoints, Color=colorsPointCloud);
    
end