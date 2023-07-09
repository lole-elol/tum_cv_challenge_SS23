function [models, pc] = pipeline(images, camParams, roomHeigth)
% PIPELINE main function of the pipeline triggered by the GUI
% perforem teh computation of the 3D point cloud and the detection of the models
% input:
%   images: cell array of images
%   camParams: camera parameters
%   scalingFactor: scaling factor for the point cloud
% output:
%   models: cell array of detected models
%   pc: point cloud


% load hyper parameters
load("config/paramsV1.mat");

% reconstruct 3D point cloud from images
[pointCloudInstance,~,~] = logic.reconstruct3DMultiview(images, camParams, reconstruction, progressdlg=progress);


%% Remove outliers
pc = logic.pointcloud.filter(pointCloudInstance, detection.outlierDist);

%% Detect floor and ceiling
[~, pc, pcFloor, floorPlane] = logic.pointcloud.groundPlane(pc);
pc = removeInvalidPoints(pc);

% Rotate point cloud so that floor is horizontal
pc = logic.pointcloud.rotate(pc, floorPlane.Normal);

[ceilingPlane, pcCeiling, pc] = logic.pointcloud.ceilPlane(pc, maxDistance=detection.ceilingDist, percentage=detection.ceilingPercentile, refVector=floorPlane.Normal, windowSize= detection.ceilingWindowSize);

% calculate scaling factor
scalingFactor = logic.pointcloud.scalingFactorFromRoomHeight(floorPlane, ceilingPlane, roomHeigth);

% scale point cloud via scaling factor
tScaling = affinetform3d([scalingFactor, 0, 0, 0; 0, scalingFactor, 0, 0; 0, 0, scalingFactor, 0; 0, 0, 0, 1]);
pc = pctransform(pointCloudInstance, tScaling);



% detect models planes and Cuboids
[models, pc, ~] = logic.modelDetection(pc, detection);


end