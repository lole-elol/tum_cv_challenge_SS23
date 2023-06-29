function pc = loadData(path)
% LOADDATA - load point cloud from massaged .txt file
% Inputs:
%   path - path to .txt file
% Outputs:
%   pc - point cloud object

data = readmatrix(path);
pc = pointCloud(data(:,2:4));

end