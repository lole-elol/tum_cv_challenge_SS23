function [index] = sortNearestNeighbour(similarityMatrix)
%SORTNEARESTNEIGHBOUR Sort the features by nearest neighbour algorithm
%
% Input:
%   similarityMatrix:  n x n matrix of similarities between features
%
% Output:
%   index:      n x 1 vector of indices of the sorted features

% find starting point

N = size(similarityMatrix, 1);

index = zeros(N, 1);

[~, index(1)] = max(sum(similarityMatrix));
similarityMatrix(:, index(1)) = inf;

% find nearest neighbour
for i = 2:N

    features = similarityMatrix(index(i-1), :);
    [dist, nextIdx] = min(features);

    %disp('Nearest neighbour: ' + string(nextIdx) + ' - Feature: ' + string(dist))

    index(i) = nextIdx;
    similarityMatrix(:, index(i)) = inf;
end

end