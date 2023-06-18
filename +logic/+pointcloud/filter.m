function [pc] = filter(pc, relativeMaxDist)
% FILTER - filter outliers from point cloud using z-score
%
% Inputs:
%   pc: point cloud
%   relativeMaxDist: maximum distance from the model to consider an inlier in standard deviations
%
% Outputs:
%   pc: filtered point cloud


points = pc.Location;

% Calculate z-scores for each column
z_scores = zscore(points);

% Find rows where any column's z-score exceeds the threshold
outlier_rows = any(abs(z_scores) > relativeMaxDist, 2);

% Remove outlier rows from matrix A
inliers = points(~outlier_rows, :);

% Create a point cloud containing only the inliers
pc = pointCloud(inliers);

end