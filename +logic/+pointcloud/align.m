function pc = align(pc)
% ALIGN Align point cloud parallel to the ground plane.
%
% Inputs:
%   pc: Point cloud.

% Compute the current rotation of the point cloud.
coeff = pca(pc.Location);
R = inv(coeff);

if det(R) < 0
    Rx = R(:, 1);
    R(:, 1) = R(:, 2);
    R(:, 2) = Rx;
end

% Rotate the point cloud.
tform = rigidtform3d(R, [0 0 0]);
pc = pctransform(pc, tform);