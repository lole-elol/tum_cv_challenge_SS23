function [pcSeg, idx, numClusters, pcRemaining] = segmentation(pc, minDist, minN)
% SEGMENTATION Segments a point cloud into clusters
% Inputs:
%   pc: point cloud
%   minDist: minimum distance between clusters
%   minN: minimum number of points in a cluster
% Outputs:
%   pcSeg: segmented point cloud
%   idx: indices of the segments
%   numClusters: number of clusters
%   pcRemaining: points that are not in any cluster

[labels, numClusters] = pcsegdist(pc, minDist);

pcSeg = cell(numClusters, 1);
pcRemaining = pointCloud([0,0,0]); % initialize "empty" point cloud


idx = cell(numClusters, 1);

for i = 1:numClusters
    tmp_idx = find(labels == i);

    if length(tmp_idx) >= minN
        idx{i} = tmp_idx;
        pcSeg{i} = select(pc, idx{i});

        % different random color for each cluster
        pcSeg{i}.Color = repmat(rand(1, 3), length(tmp_idx), 1);
    else
        pcRemaining = pcmerge(pcRemaining, select(pc, tmp_idx), 1);
    end
end

% remove first (empty) location
pcRemaining = select(pcRemaining, 2:pcRemaining.Count);

is_empty = cellfun(@isempty, pcSeg);
idx = idx(~is_empty);
pcSeg = pcSeg(~is_empty);

numClusters = length(pcSeg);

end