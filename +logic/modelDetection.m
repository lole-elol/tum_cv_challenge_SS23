function [models, pc, pcRemaining] = modelDetection(pc, varargin)
% MODELDETECTION - Detect objects in a room described by a point cloud.
%
% Inputs:
%   pc - Point cloud of the room.
%   outlierDist = 3: Distance threshold for outlier removal in standard deviations
%   clusterDist = 0.1: Distance threshold for clustering
%   clusterPoinst = 10: Minimum number of points in a cluster
%   ceilingPercentile = 0.2: Percentile of points to use for ceiling detection
%   ceilingDist = 0.2: Maximum distance between points for ceiling detection
%   cuboidVolume = 0.1: Minimum volume of a cuboid
%   cuboidOverlap = 0.9: Percentage of points inside cuboid to consider it overlapping
%   windowSize = 5: Size of window for ceiling detection
%
% Outputs:
%   models: Cell array of detected models (floor, ceiling, cuboids)
%   pc: Point cloud with outliers and detected planes removed
%   pcRemaining: Cell array of point clouds remaining after each step (floor points, ceiling points, points not in clusters, cluster not in cuboids)

%% Parse and validate input
validatePosScalar = @(x) isnumeric(x) && isscalar(x) && x >= 0;
validatePosInt = @(x) validatePosScalar(x) && isinteger(x);
validatePercentile = @(x) validatePosScalar(x) && x <= 1;

p = inputParser;
p.addParameter('outlierDist', 3, validatePosScalar);
p.addParameter('clusterDist', 0.1, validatePosScalar);
p.addParameter('clusterPoints', 10, validatePosInt);
p.addParameter('ceilingPercentile', 0.2, validatePercentile);
p.addParameter('ceilingDist', 0.2, validatePosScalar);
p.addParameter('cuboidVolume', 0.1, validatePosScalar);
p.addParameter('cuboidOverlap', 0.9, validatePercentile);
p.addParameter('windowSize', 5, validatePosInt);
p.parse(varargin{:});

outlierDist = p.Results.outlierDist;
clusterDist = p.Results.clusterDist;
clusterPoints = p.Results.clusterPoints;
ceilingPercentile = p.Results.ceilingPercentile;
ceilingDist = p.Results.ceilingDist;
cuboidVolume = p.Results.cuboidVolume;
cuboidOverlap = p.Results.cuboidOverlap;
windowSize = p.Results.windowSize;

%% Remove outliers
pc = logic.pointcloud.filter(pc, outlierDist);

%% Detect floor and ceiling
[~, pc, pcFloor, floorPlane] = logic.pointcloud.groundPlane(pc);

[ceilingPlane, pcCeiling, pc] = logic.pointcloud.ceilPlane(pc, maxDistance=ceilingDist, percentage=ceilingPercentile, refVector=floorPlane.Normal, windowSize=windowSize);

%% Detect cuboids
[seg, ~, ~, pcSegRemaining] = logic.pointcloud.segmentation(pc, clusterDist, clusterPoints);
[cuboids, ~, segRemaining] = logic.pointcloud.fitCuboids(seg, cuboidVolume, removeOverlapping=true, overlapThreshold=cuboidOverlap, mergeOverlapping=false);

%% Set outputs
pcRemaining = {
    pcFloor;
    pcCeiling;
    pcSegRemaining;
    segRemaining;
};

models = {
    floorPlane;
    ceilingPlane;
    cuboids;
};

end