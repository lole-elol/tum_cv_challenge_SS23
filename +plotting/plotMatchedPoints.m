function plotMatchedPoints(image_1,image_2, matched_points)
    % PLOTMATCHEDPOINTS Plot the matched points on the images
    % Input:
    %   image_1, images_2: the two images to be plotted
    %   matched_points: the matched points on the two images. is an array of matched points
    % Output:
    %   None
    figure;
    matched_points1 = matched_points(:,1:2);
    matched_points2 = matched_points(:,3:4);
    showMatchedFeatures(image_1, image_2, matched_points1, matched_points2);
    legend("matched points 1","matched points 2");