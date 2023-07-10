function [models, pc, scalingFactor] = pipeline(images, camParams, roomHeigth)
% PIPELINE main function of the pipeline triggered by the GUI
% perforem teh computation of the 3D point cloud and the detection of the models
% input:
%   images: cell array of images
%   camParams: camera parameters
%   roomHeigth: room height in an arbitrary distance unit
% output:
%   models: cell array of detected models
%   pc: point cloud
%   scalingFactor: scaling factor of the point cloud


%% load hyper parameters
load("config/paramsV3.mat");

%% ================== 3D Reconstruction ==================
disp("===== 3D Reconstruction =====");
[pointCloudInstance,~,~] = logic.reconstruct3DMultiview(images, camParams, reconstruction);

%% ================= Point Cloud Scaling ==================
disp("===== Point Cloud Scaling =====");

% Remove outliers
pc = logic.pointcloud.filter(pointCloudInstance, detection.outlierDist);

% Align point cloud
pc = logic.pointcloud.align(pc);

% Detect floor and ceiling
[~, ~, ~, floorPlane] = logic.pointcloud.groundPlane(pc, maxDistance=detection.floorDist);

% Rotate point cloud so that floor is horizontal
pc = logic.pointcloud.rotate(pc, floorPlane.Normal);

% Detect ceiling
[ceilingPlane, ~, ~] = logic.pointcloud.ceilPlane(pc, maxDistance=detection.ceilingDist, percentage=detection.ceilingPercentile, refVector=floorPlane.Normal, windowSize=detection.ceilingWindowSize);

% calculate scaling factor
scalingFactor = logic.pointcloud.scalingFactorFromRoomHeight(ceilingPlane, floorPlane, roomHeigth);

% Scale point cloud
%pc = logic.pointcloud.scale(pc, scalingFactor);

%% ================== Model Detection ==================
disp("===== Model Detection =====");
[models, ~, ~] = logic.modelDetection(pc, detection, scalingFactor=1, preprocess=false);

end