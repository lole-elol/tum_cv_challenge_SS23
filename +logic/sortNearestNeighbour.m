function [index] = sortNearestNeighbour(features)
%SORTNEARESTNEIGHBOUR Sort the features by nearest neighbour algorithm
%
% Input:
%   features:   d x n matrix of features (n: number of features, d: dimension of features)
%
% Output:
%   index:      n x 1 vector of indices of the sorted features

% find starting point
index = zeros(size(features, 2), 1);
[~, index(1)] = min(vecnorm(features, 2, 1));

indexUnsorted = 1:size(features, 2);

% find nearest neighbour
for i = 2:size(features, 2)
    remaining = features(:, ~ismember(1:size(features, 2), index(1:i-1)));
    remainingIndex = indexUnsorted(~ismember(1:size(features, 2), index(1:i-1)));

    featDiff = remaining - features(:, index(i-1));

    [~, nextIdx] = min(vecnorm(featDiff));

    index(i) = remainingIndex(nextIdx);
end

end