function scaledImage = resampleImage(inputImage,scale)
    % RESAMPLEIMAGE - This function samples down the input image to a desired size    
    % Inputs:
    %   inputImage: either RGB (MxNx3) or grayscale (MxN) image
    %   scale: scaling factor    
    % Outputs:
    %   croppedImage: the sampled image with the desired size
    
    sz = size(inputImage(:,:,1));
    xg = 1:sz(1);
    yg = 1:sz(2);
    
    F = griddedInterpolant({xg,yg},double(inputImage));
    xq = (0:1/scale:sz(1)-1/scale)';
    yq = (0:1/scale:sz(2)-1/scale)';
    scaledImage = uint8(F({xq,yq}));
end