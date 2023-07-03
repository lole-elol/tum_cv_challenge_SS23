function scalingFactor = scalingFactorFromHeight(ceilingPlane, floorPlane, height)
% SCALINGFACTORFROMHEIGHT  Compute the scaling factor of the point cloud based
% on the height of the room using the geometric models of the ceiling plane and floor plane
% as well as the users input height.
%
% Input:
%   ceilingPlane - planeModel of the ceiling plane model
%   floorPlane - planeModel of the floor plane model
%   height - height of the room as input by the user in meters
% Output:
%   scalingFactor - scaling factor for the point cloud

% nVecCeiling = ceilingPlane.Normal;
% nVecFloor = floorPlane.Normal;

abcdCeiling = ceilingPlane.Parameters;
abcdFloor = floorPlane.Parameters;


% Compute the height of the room from the geometric models
heightCeiling = -abcdCeiling(4) / norm(abcdCeiling(1:3));
heightFloor = -abcdFloor(4) / norm(abcdFloor(1:3));

% heightCeiling = -abcdCeiling(4) / nVecCeiling(3);
% heightFloor = -abcdFloor(4) / nVecFloor(3);

% Compute the scaling factor
scalingFactor = height / (heightCeiling - heightFloor);


end