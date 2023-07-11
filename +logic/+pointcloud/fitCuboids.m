function [cuboids, segUsed, segRemaining] = fitCuboids(pcSeg, minVolume, varargin)
%FITCUBOIDS - Fit cuboids to point cloud clusters
%
% Inputs
%  pcSeg: Point cloud clusters
%  minVolume: Minimum volume of cuboid
%  minInliers = 3: Minimum range of inlier cuboids by standard deviation
%  removeOverlapping = false: If true, remove overlapping cuboids
%  overlapThreshold = 0.9: Percentage of points inside cuboid to consider it overlapping
%  mergeOverlapping = false: If true, merge overlapping point cloud clusters
%
% Outputs
%  cuboids: Cuboids
%  segUsed: Segments used to fit cuboids
%  segRemaining: Segments not used to fit cuboids

%% Parameters
p = inputParser;
addParameter(p, 'removeOverlapping', false);
addParameter(p, 'overlapThreshold', 0.9);
addParameter(p, 'minInliers', 3);
addParameter(p, 'mergeOverlapping', false);
parse(p, varargin{:});

removeOverlapping = p.Results.removeOverlapping;
overlapThreshold = p.Results.overlapThreshold;
minInliers = p.Results.minInliers;
mergeOverlapping = p.Results.mergeOverlapping;

%% Fit cuboids
cuboids = cell(length(pcSeg), 1);
volumes = zeros(length(pcSeg), 1);

for i = 1:length(pcSeg)
    cuboid = pcfitcuboid(pcSeg{i});
    volumes(i) = cuboid.Dimensions(1) * cuboid.Dimensions(2) * cuboid.Dimensions(3);

    if volumes(i) >= minVolume
        cuboids{i} = cuboid;
    end
end

usedIdx = ~cellfun(@isempty, cuboids);

cuboids = cuboids(usedIdx);
volumes = volumes(usedIdx);

segUsed = pcSeg(usedIdx);
segRemaining = pcSeg(~usedIdx);

%% Remove outliers
if length(cuboids) > 1
  volumesStd = std(volumes);
  volumesMean = mean(volumes);

  inliers = volumes > volumesMean - volumesStd * minInliers & volumes < volumesMean + volumesStd * minInliers;

  cuboids = cuboids(inliers);

  segRemainingTmp = segUsed(~inliers);
  segUsed = segUsed(inliers);

  segRemaining = [segRemaining; segRemainingTmp];
end

%% Check for overlapping cuboids
if removeOverlapping
  overlapping = false(length(cuboids), 1);

  for i = 1:length(cuboids)
    pcCuboid = segUsed{i};

    for j = 1:length(cuboids)
      if i == j
        continue;
      end

      indices = findPointsInsideCuboid(cuboids{j}, pcCuboid);

      percentage = length(indices) / pcCuboid.Count;

      if percentage >= overlapThreshold
        overlapping(i) = true;

        if mergeOverlapping
          pcSeg{j} = pcmerge(pcSeg{j}, pcCuboid, 1);
        end
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