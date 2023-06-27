function plotMatchedPoints(image1,image2, matchedPoints)
    % PLOTMATCHEDPOINTS Plot the matched points on the images
    % Input:
    %   image1, images_2: the two images to be plotted
    %   matchedPoints: the matched points on the two images. is an array of matched points
    % Output:
    %   None
    figure;
    matchedPoints1 = matchedPoints(:,1:2);
    matchedPoints2 = matchedPoints(:,3:4);
    showMatchedFeatures(image1, image2, matchedPoints1, matchedPoints2);
    legend("matched points 1","matched points 2");