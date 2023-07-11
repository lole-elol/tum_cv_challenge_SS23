function pc = scale(pc, scalingFactor)
% SCALE  Scale a point cloud by a given factor
%
% Inputs:
%   pc: Point cloud
%   scalingFactor: Scaling factor
%
% Outputs:
%   pc: Scaled point cloud

tScaling = affinetform3d([scalingFactor, 0, 0, 0; 0, scalingFactor, 0, 0; 0, 0, scalingFactor, 0; 0, 0, 0, 1]);
pc = pctransform(pc, tScaling);