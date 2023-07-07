function [element, idx] = kNearestNeighbour(features, startIdx, k)
% KNEARESTNEIGHBOUR - Get the kth nearest neighbour of a element in a feature vector
%
% Inputs:
%    features: A matrix of n features with m dimensions (m x n)
%    startIdx: The index of the element to find the kth nearest neighbour of
%    k: The kth nearest neighbour to find
%
% Outputs:
%   element: The kth nearest neighbour of the element at startIdx
%   idx: The index of the kth nearest neighbour

remaining = features(:, ~ismember(1:size(features, 2), startIdx));

indices = 1:size(features, 2);
remainingIdx = indices(~ismember(1:size(features, 2), startIdx));

distances = remaining - features(:, startIdx);
[~, indices] = mink(vecnorm(distances), k);

idx = remainingIdx(indices(end));
element = features(:, idx);

end
