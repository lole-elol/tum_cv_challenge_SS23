function [geoPlaneModel, geoPlanePC, remainingPC] = ceilPlane(pc, varargin)
% CEILPLANE  Fit a plane to the ceiling of a room
%
% Input:
% pc: point cloud
% maxDistance = 0.2: Maximum distance from an inlier point to the plane.
% percentage = 0.3: Percentage of points to use for plane fitting
% refVector = [0 0 1]: Reference vector for plane fitting

% OUTPUTS
% geoPlaneModel: plane model
% geoPlanePC: point cloud of the plane
% remainingPC: point cloud of the remaining points


p = inputParser;
addParameter(p, 'maxDistance', 0.2, @isnumeric);
addParameter(p, 'percentage', 0.3, @isnumeric);
addParameter(p, 'refVector', [0 0 1]);
parse(p, varargin{:});
maxDistance = p.Results.maxDistance;
percentage = p.Results.percentage;
refVector = p.Results.refVector;

%  sort the poitns by z
[~,idx] = sort(pc.Location(:,3));
% get the top 30% indices
top30 = idx(end-floor(percentage*length(idx)):end);
% fit a plane to the top 30% points
top30PC = select(pc, top30);
[geoPlaneModel,inlierIndices,outlierIndices] = pcfitplane(top30PC, maxDistance, refVector, maxAngularDistance=1);
% get the remaining points
remainingPCTop30 = select(top30PC,outlierIndices);
remainingPC = select(pc, idx(1:floor((1-percentage)*length(idx))));
remainingPC = pcmerge(remainingPC, remainingPCTop30, 0.001);
% get the plane points
geoPlanePC = select(top30PC,inlierIndices);

end

