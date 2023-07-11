function [idx, element] = kNearestNeighbour(similarity, k)
% KNEARESTNEIGHBOUR - Get the kth nearest neighbour of a element in a feature vector
%
% Inputs:
%    similarity: A vector of similarity values between the element at startIdx and all other elements
%    startIdx: The index of the element to find the kth nearest neighbour of
%    k: The kth nearest neighbour to find
%
% Outputs:
%   idx: The index of the kth nearest neighbour
%   element: The kth nearest neighbour of the element at startIdx

[dist, indices] = mink(similarity, k+1);

element = dist(k+1);
idx = indices(k+1);
end
