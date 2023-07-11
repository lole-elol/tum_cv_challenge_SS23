function pc = filterFarOutliers(pc, camPoses, maxDistance)
% FILTERFAROUTLIERS Filter outliers that are far from the camera (eg. in Italy)
%
% Inputs:
%   pc: pointCloud object
%   camPoses: cameraPoses object
%   maxDistance: maximum distance from camera to keep points in standard deviation
%
% Outputs:
%   pc: filtered pointCloud object

% Get camera poses as Matrix
posesCell = [];
for el=camPoses.AbsolutePose
  posesCell=[posesCell, {el.Translation}];
end
posesCell = cellfun(@(x) x', posesCell, 'UniformOutput', false);
poses = cell2mat(posesCell);

% Mean poses and standard deviation
meanPoses = mean(poses');
stdPoses = std(poses');
maxDistAbs = maxDistance * stdPoses;


% Filter points that are far from the camera
pc = select(pc, find(pc.Location(:,1) < meanPoses(1) + maxDistAbs(1) & ...
                     pc.Location(:,1) > meanPoses(1) - maxDistAbs(1) & ...
                     pc.Location(:,2) < meanPoses(2) + maxDistAbs(2) & ...
                     pc.Location(:,2) > meanPoses(2) - maxDistAbs(2) & ...
                     pc.Location(:,3) < meanPoses(3) + (maxDistAbs(2) + maxDistAbs(1))/2 & ...
                     pc.Location(:,3) > meanPoses(3) - (maxDistAbs(2) + maxDistAbs(1))/2 ...
                    ));
end