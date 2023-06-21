function [vSet, prevFeatures, prevPoints] = createViewSet(image_1, ~, varargin)
    p = inputParser;
    p.addOptional('numOctaves', 15);
    p.addOptional('roi_border', 20);
    p.parse(varargin{:});
    numOctaves = p.Results.numOctaves;
    roi_border = p.Results.roi_border;

    roi = [roi_border, roi_border, size(image_1, 2)- 2*roi_border, size(image_1, 1)- 2*roi_border];
    prevPoints = detectSURFFeatures(image_1, NumOctaves=numOctaves, ROI=roi);
    prevFeatures = extractFeatures(image_1, prevPoints);

    % Create an empty imageviewset object to manage the data associated with each view.
    vSet = imageviewset;

    % Add the first view. Place the camera associated with the first view and the origin, oriented along the Z-axis.
    viewId = 1;
    vSet = addView(vSet, viewId, rigidtform3d, Points=prevPoints);
end
