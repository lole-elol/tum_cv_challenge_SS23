function [pointCloudInstance, camPoses] = transformScene(pointCloudInstance, camPoses, tform)
% TRANSFORMSCENE Transform a point cloud and camera poses based on an affine transform
%   [pointCloudInstance, camPoses] = transformScene(pointCloudInstance, camPoses, tform)
%   transforms a point cloud and camera poses based on an affine transform.
%   The point cloud and camera poses are transformed in place.
%
% Inputs:
%   pointCloudInstance - a pointCloud object
%   camPoses - a table containing camera poses
%   tform - an affine transform applied to the point cloud and camera (scaling/rotating/translation...)
% Outputs:
%   pointCloudInstance - a pointCloud object
%   camPoses - a table containing camera poses

pointCloudInstance = pctransform(pointCloudInstance, tform);
% Transform the camera poses to the new coordinate system
for i = 1:height(camPoses)
    if det(tform.A) == 1
        camPoses.AbsolutePose(i) = rigidtform3d(tform.A * camPoses.AbsolutePose(i).A);
    else
        camPoses.AbsolutePose(i) = rigidtform3d(tform.A * camPoses.AbsolutePose(i).A * tform.A^-1);  % remove the scaling
    end
end
end
