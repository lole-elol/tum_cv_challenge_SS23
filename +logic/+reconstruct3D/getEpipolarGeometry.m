function [E, relPose, status, inliers] = getEpipolarGeometry(matchedPoints1, matchedPoints2, cameraParams, varargin)
% GETEPIPOLARGEOMETRY Get the epipolar geometry between two images. If the estimation fails, relpose is set to identity.
% Input:
%   matchedPoints1, matchedPoints2: matched points between two images
%   cameraParams: camera parameters
%   eMaxDistance = 5: max distance for RANSAC
%   eConfidence = 99.6: confidence for RANSAC
%   eMaxNumTrials = 1000: max number of trials for RANSAC
%   eValidPointFraction = 0.8: valid point fraction for RANSAC
% Output:
%   E: essential matrix
%   relPose: relative pose between two images
%   status: 0 if success, else if failed
%   inliers: inliers for the estimation
p = inputParser;
p.addOptional('eMaxDistance', 5);
p.addOptional('eConfidence', 99.6);
p.addOptional('eMaxNumTrials', 100000);
p.addOptional('eValidPointFraction', 0.8);
p.parse(varargin{:});
eMaxDistance = p.Results.eMaxDistance;
eConfidence = p.Results.eConfidence;
eMaxNumTrials = p.Results.eMaxNumTrials;
eValidPointFraction = p.Results.eValidPointFraction;

if ~isnumeric(matchedPoints1)
    matchedPoints1 = matchedPoints1.Location;
end
if ~isnumeric(matchedPoints2)
    matchedPoints2 = matchedPoints2.Location;
end

[E, inliers, status] = estimateEssentialMatrix(matchedPoints1, matchedPoints2, cameraParams, ...
MaxDistance=eMaxDistance, Confidence=eConfidence, MaxNumTrials=eMaxNumTrials);

[~, warnId] = lastwarn();

if strcmp(warnId, 'vision:ransac:maxTrialsReached')
    disp('Max number of trials reached');
    status = 1;

    lastwarn('');
end

if status ~= 0
    disp('Failed to estimate essential matrix');
    relPose = rigidtform3d(eye(4));
else
    inlierPoints1 = matchedPoints1(inliers, :);
    inlierPoints2 = matchedPoints2(inliers, :);
    relPose = estrelpose(E, cameraParams.Intrinsics, cameraParams.Intrinsics, inlierPoints1, inlierPoints2);
    % if validPointFraction > eValidPointFraction % TODO: check if this is a good threshold
    %         relPose = rigidtform3d;
    %         % return;  % TODO: check this handling, should we do this here? return some type of status
    % end
end
end