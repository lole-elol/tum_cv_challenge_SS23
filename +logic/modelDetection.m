function [models, pc, pcRemaining] = modelDetection(pc, varargin)
% MODELDETECTION - Detect objects in a room described by a point cloud.
%
% Inputs:
%   pc - Point cloud of the room.
%   outlierDist = 3: Distance threshold for outlier removal in standard deviations
%   clusterDist = 0.1: Distance threshold for clustering in standard deviations
%   clusterPercentile = 0.0006: Minimum percentage of points in a cluster
%   clusterDenoise = 0.5: Distance threshold for denoising clusters in standard deviations
%   clusterDenoiseNeighbours = 10: number of neighbours for denoising clusters
%   ceilingPercentile = 0.2: Percentile of points to use for ceiling detection
%   ceilingDist = 0.2: Maximum distance between points for ceiling detection
%   ceilingWindowSize = 5: Size of window for ceiling detection
%   floorDist = 0.2: Maximum distance between points for floor detection
%   cuboidVolume = 0.1: Minimum volume of a cuboid
%   cuboidInlier = 3: Minimum percentage of inlier cuboids by volume in standard deviations
%   cuboidOverlap = 0.9: Percentage of points inside cuboid to consider it overlapping
%   scalingFactor = 1: Scaling factor for point cloud
%   preprocess = true: Whether to preprocess the point cloud (remove outliers, align)
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
p.addParameter('clusterPercentile', 0.0006, validatePercentile);
p.addParameter('clusterDenoise', 0.5, validatePosScalar);
p.addParameter('clusterDenoiseNeighbours', 10, validatePosScalar);
p.addParameter('ceilingPercentile', 0.2, validatePercentile);
p.addParameter('ceilingDist', 0.2, validatePosScalar);
p.addParameter('ceilingWindowSize', 5, validatePosScalar);
p.addParameter('floorDist', 0.2, validatePosScalar);
p.addParameter('cuboidVolume', 0.1, validatePosScalar);
p.addParameter('cuboidInlier', 3, validatePosScalar);
p.addParameter('cuboidOverlap', 0.9, validatePercentile);
p.addParameter('scalingFactor', 1, validatePosScalar);
p.addParameter('preprocess', true, @islogical);
p.parse(varargin{:});

outlierDist = p.Results.outlierDist;
clusterDist = p.Results.clusterDist;
clusterPercentile = p.Results.clusterPercentile;
clusterDenoise = p.Results.clusterDenoise;
clusterDenoiseNeighbours = p.Results.clusterDenoiseNeighbours;
ceilingPercentile = p.Results.ceilingPercentile;
ceilingDist = p.Results.ceilingDist;
ceilingWindowSize = p.Results.ceilingWindowSize;
floorDist = p.Results.floorDist;
cuboidVolume = p.Results.cuboidVolume;
cuboidInlier = p.Results.cuboidInlier;
cuboidOverlap = p.Results.cuboidOverlap;
scalingFactor = p.Results.scalingFactor;
preprocess = p.Results.preprocess;

%% Scale absolute parameters
ceilingDist = ceilingDist * scalingFactor;
cuboidVolume = cuboidVolume * scalingFactor;
floorDist = floorDist * scalingFactor;

if preprocess
    %% Remove outliers
    pc = logic.pointcloud.filter(pc, outlierDist);

    %% Align point cloud
    pc = logic.pointcloud.align(pc);
end

%% Detect floor and ceiling

[~, pc, pcFloor, floorPlane] = logic.pointcloud.groundPlane(pc, floorDist);

pc = removeInvalidPoints(pc);
disp('rotating cloud')
% Rotate point cloud so that floor is horizontal
pc = logic.pointcloud.rotate(pc, floorPlane.Normal);
disp('fit the ceiling')
[ceilingPlane, pcCeiling, pc] = logic.pointcloud.ceilPlane(pc, maxDistance=ceilingDist, percentage=ceilingPercentile, refVector=floorPlane.Normal, windowSize=ceilingWindowSize);
disp('detect cuboids')
%% Detect cuboids
[seg, ~, ~, pcSegRemaining] = logic.pointcloud.segmentation(pc, minDist=clusterDist, minP=clusterPercentile, denoise=true, denoiseNeighbours=clusterDenoiseNeighbours, denoiseThreshold=clusterDenoise);
[cuboids, ~, segRemaining] = logic.pointcloud.fitCuboids(seg, cuboidVolume, minInliers=cuboidInlier, removeOverlapping=true, overlapThreshold=cuboidOverlap, mergeOverlapping=false);

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
disp('finished models')
end