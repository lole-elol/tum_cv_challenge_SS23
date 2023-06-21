function [geoPlaneModel, geoPlanePC, remainingPC] = ceilPlane2(pc, varargin)
% CEILPLANE  Fit a plane to the ceiling of a room
%
% Input:
% pc: point cloud
% maxDistance = 0.2: Maximum distance from an inlier point to the plane.
% percentage = 0.3: Percentage of points to use for plane fitting
% windowSize = 5: Size of the filter window in maxDistance
% refVector = [0 0 1]: Reference vector for plane fitting

% OUTPUTS
% geoPlaneModel: plane model
% geoPlanePC: point cloud of the plane
% remainingPC: point cloud of the remaining points

p = inputParser;
addParameter(p, 'maxDistance', 0.2, @isnumeric);
addParameter(p, 'percentage', 0.3, @isnumeric);
addParameter(p, 'refVector', [0 0 1]);
addParameter(p, 'windowSize', 5, @isnumeric);
parse(p, varargin{:});
maxDistance = p.Results.maxDistance;
percentage = p.Results.percentage;
refVector = p.Results.refVector;
windowHalfSize = p.Results.windowSize;


[~,idx] = sort(pc.Location(:,3));
% get the top 30% indices
top30 = idx(end-floor(percentage*length(idx)):end);
% fit a plane to the top 30% points
top30PC = select(pc, top30);
top30Z = top30PC.Location(:,3);
top30Y = top30PC.Location(:,2);
top30X = top30PC.Location(:,1);

top30_zmin = min(top30PC.Location(:,3));
top30_zmax = max(top30PC.Location(:,3));

step_size = maxDistance/(2 * windowHalfSize + 1);

% Gaussian window
window = gausswin(windowHalfSize*2+1)/sum(gausswin(windowHalfSize*2+1));

z_grid =  top30_zmin:step_size:top30_zmax;
pdf_z = zeros(1,length(z_grid)-1);
pdf_x = zeros(1,length(z_grid)-1);
pdf_y = zeros(1,length(z_grid)-1);

for i= 1:length(z_grid)-1
    for j= -(min(windowHalfSize, i-1)):min(windowHalfSize, length(z_grid)-i-1)
        logicalRange = top30Z > z_grid(i+j) & top30Z <= z_grid(i+j+1);

        pdf_z(i) = pdf_z(i) + window(j+windowHalfSize+1)*length(top30Z(logicalRange));
        pdf_x(i) = pdf_x(i) + window(j+windowHalfSize+1)*std(top30X(logicalRange));
        pdf_y(i) = pdf_y(i) + window(j+windowHalfSize+1)*std(top30Y(logicalRange));
    end
end

pdfZnorm = pdf_z/max(pdf_z);
pdfPlane = (pdf_x+pdf_y)/2;

pdfZmax = islocalmax(pdfZnorm) & (pdfZnorm > 0.1);
pdfPlaneZmax = pdfPlane .* pdfZmax;

[~, planeSpaceIdx] = max(pdfPlaneZmax)

zPlane = (z_grid(planeSpaceIdx) + step_size/2)
planeSpace = [zPlane - maxDistance/2, zPlane + maxDistance/2];

pcRoiIdx = find(pc.Location(:,3) > planeSpace(1) & pc.Location(:,3) < planeSpace(2));
pcRoi = select(pc, pcRoiIdx);

% Find the plane
%! Should be replaced with custom generated plane because of performance
[model,~,~] = pcfitplane(pcRoi, maxDistance, refVector);
geoPlaneModel = model;
geoPlanePC = pcRoi;
remainingPC = select(pc, setdiff(1:pc.Count, pcRoiIdx));

%figure
%hold on
%
%z_grid_ = z_grid(1:end-1);
%
%plot(z_grid_, pdfZnorm)
%% plot(z_grid(1:end-1),pdf_x )
%% plot(z_grid(1:end-1),pdf_y )
%plot(z_grid_, pdfPlane)
%plot(z_grid_(pdfZmax), pdfPlane(pdfZmax), 'o')
%plot(z_grid_(pdfZmax), pdfZnorm(pdfZmax), 'o')
%plot(z_grid_(planeSpaceIdx), pdfPlaneZmax(planeSpaceIdx), '*')
%hold off

end