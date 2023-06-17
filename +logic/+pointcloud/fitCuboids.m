function [cuboids, segUsed, segRemaining] = fitCuboids(pcSeg, minVolume, varargin)
%FITCUBOIDS - Fit cuboids to point cloud clusters
%
% Inputs
%  pcSeg: Point cloud clusters
%  minVolume: Minimum volume of cuboid
%  removeOverlapping = false: If true, remove overlapping cuboids
%  overlapThreshold = 4: Minimum number of corners inside cuboid to be considered overlapping
%
% Outputs
%  cuboids: Cuboids
%  segUsed: Segments used to fit cuboids
%  segRemaining: Segments not used to fit cuboids

%% Parameters
p = inputParser;
addParameter(p, 'removeOverlapping', false);
addParameter(p, 'overlapThreshold', 4);
parse(p, varargin{:});

removeOverlapping = p.Results.removeOverlapping;
overlapThreshold = p.Results.overlapThreshold;

%% Fit cuboids
cuboids = cell(length(pcSeg), 1);

for i = 1:length(pcSeg)
    cuboid = pcfitcuboid(pcSeg{i});
    volume = cuboid.Dimensions(1) * cuboid.Dimensions(2) * cuboid.Dimensions(3);

    if volume >= minVolume
        cuboids{i} = cuboid;
    end
end

usedIdx = ~cellfun(@isempty, cuboids);

cuboids = cuboids(usedIdx);
segUsed = pcSeg(usedIdx);
segRemaining = pcSeg(~usedIdx);

%% Check for overlapping cuboids
if removeOverlapping
  overlapping = false(length(cuboids), 1);

  for i = 1:length(cuboids)
    cuboid_a = cuboids{i};

    for j = 1:length(cuboids)
      if i == j
        continue;
      end

      corners = pointCloud(getCornerPoints(cuboid_a));
      indices = findPointsInsideCuboid(cuboids{j}, corners);


      if length(indices) >= overlapThreshold
        [length(indices), i, j]
        overlapping(i) = true;
        break;
      end
    end

  end

  cuboids = cuboids(~overlapping);

  segRemainingTmp = segUsed(overlapping);
  segUsed = segUsed(~overlapping);

  segRemaining = [segRemaining; segRemainingTmp];
end


end