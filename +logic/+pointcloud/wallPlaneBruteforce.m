function [planes] = wallPlaneBruteforce(geoPlanes)
% WALLPLANEBRUTEFORCE retun planes wich are orthogonal to each other and to [0 0 1] aka: walls
% it is assumed that the walls of a room are orthogonal to each other, so the room has to be a rectangle
%Input:
%   geoPlanes: cell array of geometric planes
%Output:
%   planes: cell array of geometric planes

% filter out planes that are not orthogonal to each other
geoPlanes = geoPlanes(cellfun(@(x) ~isempty(x), geoPlanes));
planes = cell(1, length(geoPlanes));
for i = 1:length(geoPlanes)
    for j = 1:length(geoPlanes)
        if i ~= j
            if abs(dot(geoPlanes{i}.Normal, geoPlanes{j}.Normal)) < 0.0001
                planes{i} = geoPlanes{i};
                break;
            end
        end
    end
end

% filter out planes that are not orthogonal to [0 0 1]
planes = planes(cellfun(@(x) ~isempty(x), planes));
planes = planes(cellfun(@(x) abs(x.Normal(3)) < 0.7, planes));

end
