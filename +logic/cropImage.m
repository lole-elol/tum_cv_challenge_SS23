function croppedImage = cropImage(inputImage,desiredsize)
    % This function cropps down the input image to a desired size
    sz = size(inputImage(:,:,1));
    xg = 1:sz(1);
    yg = 1:sz(2);

    scale = sz./desiredsize;

    F = griddedInterpolant({xg,yg},double(inputImage));
    xq = (0:scale(1):sz(1)-scale(1))';
    yq = (0:scale(2):sz(2)-scale(2))';
    croppedImage = uint8(F({xq,yq}));
end