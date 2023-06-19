function croppedImage = cropImage(inputImage,desiredsize)
    % CROPIMAGE - This function cropps down the input image to a desired size    
    % Inputs:
    %   inputImage: either RGB (MxNx3) or grayscale (MxN) image
    %   desiredsize: 1x2 vector containing the desired size
    %   desiredsize elements must be stricty smaller than the corresponding
    %   elements of size(inputImage)
    % Outputs:
    %   croppedImage: the sampled image with the desired size
    if(any(size(inputImage(:,:,1))<=desiredsize))
        error('desiredsize elements must be stricty smaller than the corresponding elements of size(inputImage)');
    end    
    sz = size(inputImage(:,:,1));
    xg = 1:sz(1);
    yg = 1:sz(2);
    scale = sz./desiredsize;
    F = griddedInterpolant({xg,yg},double(inputImage));
    xq = (0:scale(1):sz(1)-scale(1))';
    yq = (0:scale(2):sz(2)-scale(2))';
    croppedImage = uint8(F({xq,yq}));
end