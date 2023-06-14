function [pcSeg, idx] = segmentation(pc, dist)
% SEGMENTATION Segments a point cloud into clusters
% pc: point cloud
% dist: distance threshold for segmentation
% pc_seg: segmented point cloud
% idx: indices of the segments


[labels, numClusters] = pcsegdist(pc, dist);

pcSeg = cell(numClusters, 1);
idx = cell(numClusters, 1);

for i = 1:numClusters
    idx{i} = find(labels == i);
    pcSeg{i} = select(pc, idx{i});
end

end