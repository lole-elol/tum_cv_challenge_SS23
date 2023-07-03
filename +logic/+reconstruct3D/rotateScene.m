function [pointCloudInstance, camPoses] = rotateScene(pointCloudInstance, camPoses, rotationMatrix)
% ROTATESCENE Rotate the point cloud and camera poses by the given rotation matrix
%   [pointCloudInstance, camPoses] = rotateScene(pointCloudInstance, camPoses, rotationMatrix)
%   rotates the point cloud and camera poses by the given rotation matrix.
%   The rotation matrix is a 3x3 matrix. The point cloud and camera poses
%   are returned as output arguments.
% Inputs:
%   pointCloudInstance - a pointCloud object
%   camPoses - a table containing camera poses
%   rotationMatrix - a 3x3 rotation matrix
% Outputs:
%   pointCloudInstance - a pointCloud object
%   camPoses - a table containing camera poses

tform = rigidtform3d(rotationMatrix, [0 0 0]);
pointCloudInstance = pctransform(pointCloudInstance, tform);
% Transform the camera poses to the new coordinate system
for i = 1:height(camPoses)
    camPoses.AbsolutePose(i) = rigidtform3d(tform.A * camPoses.AbsolutePose(i).A);
end

end
