function pc = align(pc)
% ALIGN Align point cloud parallel to the ground plane.
%
% Inputs:
%   pc: Point cloud.

% Translate the point cloud to the origin.
T = - mean(pc.Location);
tform = rigidtform3d(eye(3), T);
pc = pctransform(pc, tform);

% Compute the current rotation of the point cloud.
coeff = pca(pc.Location);

if det(coeff) < 0
    Rx = coeff(:, 1);
    coeff(:, 1) = coeff(:, 2);
    coeff(:, 2) = Rx;
end

R = inv(coeff);

% Rotate the point cloud.
tform = rigidtform3d(R, [0 0 0]);
pc = pctransform(pc, tform);