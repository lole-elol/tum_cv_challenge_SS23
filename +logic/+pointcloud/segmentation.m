function [pcSeg, idx, numClusters, pcRemaining] = segmentation(pc, varargin)
% SEGMENTATION Segments a point cloud into clusters
% Inputs:
%   pc: point cloud
%   minDist = 0.1: minimum distance between clusters in standard deviations
%   minP =  0.0006: minimum percentage of points in a cluster
%   denoise = false: denoise point cloud before segmentation
%   denoiseNeighbours = 10: number of neighbors for denoising
%   denoiseThreshold = 0.5: threshold for denoising in standard deviations
% Outputs:
%   pcSeg: segmented point cloud
%   idx: indices of the segments
%   numClusters: number of clusters
%   pcRemaining: points that are not in any cluster

%% Parameters
p = inputParser;
addParameter(p, 'minDist', 0.1);
addParameter(p, 'minP',  0.0006);
addParameter(p, 'denoise', false);
addParameter(p, 'denoiseNeighbours', 10);
addParameter(p, 'denoiseThreshold', 0.5);
parse(p, varargin{:});

minDistStd = p.Results.minDist;
minP = p.Results.minP;
denoise = p.Results.denoise;
denoiseNeighbours = p.Results.denoiseNeighbours;
denoiseThreshold = p.Results.denoiseThreshold;

%% Segmentation
% scale minDistStd and minP to absolute values
minDist = minDistStd * sqrt(vecnorm(std(pc.Location)))
minN = minP * pc.Count

% segment
[labels, numClusters] = pcsegdist(pc, minDist);

%% separate clusters
pcSeg = cell(numClusters, 1);
pcRemaining = pointCloud([0,0,0]); % initialize "empty" point cloud

idx = cell(numClusters, 1);

for i = 1:numClusters
    tmp_idx = find(labels == i);

    % only keep clusters with more than minN points
    if length(tmp_idx) >= minN
        idx{i} = tmp_idx;
        pcTmp = select(pc, idx{i});

        % denoise
        if denoise
            pcTmp = pcdenoise(pcTmp, 'NumNeighbors', denoiseNeighbours, 'Threshold', denoiseThreshold);
        end

        pcSeg{i} = pcTmp;

        % different random color for each cluster
        pcSeg{i}.Color = rand(1, 3);
    else
        pcRemaining = pcmerge(pcRemaining, select(pc, tmp_idx), 1);
    end
end

% remove first (empty) location
pcRemaining = select(pcRemaining, 2:pcRemaining.Count);

% remove empty cells
is_empty = cellfun(@isempty, pcSeg);
idx = idx(~is_empty);
pcSeg = pcSeg(~is_empty);

numClusters = length(pcSeg);

end