function pc = rotate(pc, refVector)
% ROTATE - Rotate the point cloud so the xy plane is orthogonal to the refVector
%
% Inputs:
%   pc: point cloud
%   refVector: reference vector
%
% Outputs:
%   pc: rotated point cloud

%% Get rotation Matrix
refVector = refVector/norm(refVector);
xAxis = [1 0 0];
yAxis = [0 1 0];

e3 = refVector;
e2 = yAxis - dot(yAxis, e3)*e3;
e2 = e2/norm(e2);
e1 = xAxis - dot(xAxis, e3)*e3 - dot(xAxis, e2)*e2;
e1 = e1/norm(e1);

R = [e1; e2; e3];

%% Rotate
tform = rigidtform3d(R, [0 0 0]);
pc = pctransform(pc, tform);

end