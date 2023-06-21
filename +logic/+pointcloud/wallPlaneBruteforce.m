function [planes] = wallPlaneBruteforce(geoPlanes)
% wallPlaneBruteforce filters out planes that are not orthogonal to ecach other
% or where the normal vector is not orthogonal to [ 0 0 1]
% geoPlanes: struct of planes with fileds: Parameters, Normal
% planes: struct of planes with fileds: Parameters, Normal
% tanaplh = sin(5)
% atan2(norm(cross(a,b)), dot(a,b))
% filter out planes that are not orthogonal to ecach other plane in geoPlanes
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
