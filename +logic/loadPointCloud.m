function pc = loadPointCloud(path)
% LOADPOINTCLOUD - load point cloud from massaged .txt file

data = readmatrix(path);
pc = pointCloud(data(:,2:4), "Color", data(:,5:7));

end