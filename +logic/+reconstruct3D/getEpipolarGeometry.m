function [E, relPose, status] = getEpipolarGeometry(matchedPoints1, matchedPoints2, cameraParams, varargin)
% GETEPIPOLARGEOMETRY Get the epipolar geometry between two images. If the estimation fails, relpose is set to identity.
% Input:
%   matchedPoints1, matchedPoints2: matched points between two images
%   cameraParams: camera parameters
%   eMaxDistance = 1.5: max distance for RANSAC
%   eConfidence = 99.99: confidence for RANSAC
%   eMaxNumTrials = 1000: max number of trials for RANSAC
% Output:
%   E: essential matrix
%   relPose: relative pose between two images
%   status: 0 if success, else if failed
p = inputParser;
p.addOptional('eMaxDistance', 1.5);
p.addOptional('eConfidence', 99.99);
p.addOptional('eMaxNumTrials', 1000);
p.parse(varargin{:});
eMaxDistance = p.Results.eMaxDistance;
eConfidence = p.Results.eConfidence;
eMaxNumTrials = p.Results.eMaxNumTrials;

[E, inliers, status] = estimateEssentialMatrix(matchedPoints1, matchedPoints2, cameraParams, ...
MaxDistance=eMaxDistance, Confidence=eConfidence, MaxNumTrials=eMaxNumTrials);
if status ~= 0
    relPose = rigidtform3d;
else
    inlierPoints1 = matchedPoints1(inliers);
    inlierPoints2 = matchedPoints2(inliers);
    relPose = estrelpose(E, cameraParams.Intrinsics, cameraParams.Intrinsics, inlierPoints1, inlierPoints2);    
end
end