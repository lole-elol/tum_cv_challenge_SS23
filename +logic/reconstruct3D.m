function [pointCloudInstance, relPose, matchedPoints] = reconstruct3D(image1, image2, cameraParams, varargin)
    % RECONSTRUCT3D Take a set of images and a set of camera parameters and returns a 3D point cloud of the environment.
    % Ressources:
    %   https://de.mathworks.com/help/vision/ug/structure-from-motion-from-two-views.html
    %   https://de.mathworks.com/help/vision/ug/structure-from-motion-from-multiple-views.html
    %   https://de.mathworks.com/support/search.html/answers/153348-tips-and-tricks-about-3d-scene-reconstruction.html?fq%5B%5D=asset_type_name:answer&fq%5B%5D=category:vision/stereo-vision&page=1
    % Inputs:
    %   image1, image2: The two images to reconstruct the 3D environment from
    %   cameraParams: The camera parameters of the camera used to take the images. This is a struct of type cameraParameters
    %   minQuality1 = 0.1: The minimum quality of the features to detect in the first detection
    %   minQuality2 = 0.01: The minimum quality of the features to detect in the second detection
    %   roiBorder = 200: The border around the image to ignore when detecting features
    %   eMaxDistance = 0.2: The maximum distance of a point to the epipolar line to be considered an inlier
    %   eConfidence = 99.99: The confidence of the estimated essential matrix
    %   eMaxNumTrials = 10000: The maximum number of trials to estimate the essential matrix
    %   maxReprojection_error = 100: The maximum reprojection error of a point to be considered an inlier
    % Outputs:
    %   pointCloud: The 3D point cloud of the environment
    %   relPose: The relative pose of the second camera to the first camera
    %   matchedPoints: The matched points between the two images. Mx4 matrix with the x and y coordinates of the matched points in the first and second image

    %% === 0. Parse input arguments ===
    p = inputParser;
    p.addOptional('minQuality1', 0.1);
    p.addOptional('minQuality2', 0.01);
    p.addOptional('eMaxDistance', 0.2);
    p.addOptional('eConfidence', 99.99);
    p.addOptional('eMaxNumTrials', 10000);
    p.addOptional('roiBorder', 200);
    p.addOptional('maxReprojectionError', 100);
    % p.addOptional('max_z', 100);
    p.parse(varargin{:});

    minQuality1 = p.Results.minQuality1;
    minQuality2 = p.Results.minQuality2;
    eMaxDistance = p.Results.eMaxDistance;
    eConfidence = p.Results.eConfidence;
    eMaxNumTrials = p.Results.eMaxNumTrials;
    roiBorder = p.Results.roiBorder;
    maxReprojectionError = p.Results.maxReprojectionError;
    % max_z = p.Results.max_z;
    % ==================================

    %% === 1. Preprocessing ===
    [imagePreprocessed1, imageGray1, imageCanny1] = logic.reconstruct3D.preprocessImage(image1, cameraParams);
    [imagePreprocessed2, imageGray2, imageCanny2] = logic.reconstruct3D.preprocessImage(image2, cameraParams);
    imagePreprocessed1  = imageGray1;
    imagePreprocessed2  = imageGray2;

    %% ===  2. Feature detection and matching ===
    [matchedPoints1, matchedPoints2] = logic.reconstruct3D.extractCommonFeatures(imagePreprocessed1, imagePreprocessed2, cameraParams, minQuality=minQuality1, roiBorder=0);
    
    %% === 3. Epipolar geometry: estimate essential matrix and relative pose of the cameras ===
    [E, relPose, status] = logic.reconstruct3D.getEpipolarGeometry(matchedPoints1, matchedPoints2, cameraParams, ...
        eMaxDistance=eMaxDistance, eConfidence=eConfidence, eMaxNumTrials=eMaxNumTrials);
    if status ~= 0
        error("Could not estimate the essential matrix");
    end

    %% === 4. Triangulation === 
    % Get more features from the images to generate a bigger point cloud
    [matchedPoints1, matchedPoints2] = logic.reconstruct3D.extractCommonFeatures(imagePreprocessed1, imagePreprocessed2, cameraParams, minQuality=minQuality2, roiBorder=roiBorder);
    % Triangulate the points
    pointCloudInstance = logic.reconstruct3D.getTriangulatedPoints(matchedPoints1, matchedPoints2, cameraParams, relPose, image1, maxReprojectionError);

    %% === 5. prepare output ===
    matchedPoints = [matchedPoints1.Location, matchedPoints2.Location];