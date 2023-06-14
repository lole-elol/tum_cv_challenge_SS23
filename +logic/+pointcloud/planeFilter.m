function [planes] = planeFilter(geoPlanes)
% PLANEFILTER filters out planes that are not orthogonal to ecach other
% geoPlanes: struct of planes with fileds: Parameters, Normal
% planes: struct of planes with fileds: Parameters, Normal
% tanaplh = sin(5)
% atan2(norm(cross(a,b)), dot(a,b))
% filter out planes that are not orthogonal to ecach other plane in geoPlanes
geoPlanes = geoPlanes(cellfun(@(x) ~isempty(x), geoPlanes))
planes = cell(1, length(geoPlanes));
for i = 1:length(geoPlanes)
    for j = 1:length(geoPlanes)
        if i ~= j
            if abs(dot(geoPlanes{i}.Normal, geoPlanes{j}.Normal)) < 0.1
                planes{i} = geoPlanes{i};
                break;
            end
        end
    end
    % if abs(cross(geoPlanes{i}.Normal, geoPlanes{j}.Normal)) < tanaplh
    %     planes{i} = geoPlanes{i};
    %     break;
    
    % for i = 1:length(geoPlanes)
    %     geoPlanes{i}.Normal
    % end
end
