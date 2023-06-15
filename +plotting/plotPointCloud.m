function plotPointCloud(point_cloud, cam_poses, varargin)
    % Plot the point cloud with the camera poses
    % Inputs:
    %   point_cloud - 3D point cloud as a pointCloud object
    %   cam_poses - camera poses as an array of rigid3d objects
    %   camera_size_plot_size - size of the camera in the plot
    %   pc_marker_size - size of the point cloud marker
    % Outputs:
    %   None

    % Define parser
    p = inputParser;
    p.addOptional('camera_size_plot_size', 1.0);
    p.addOptional('pc_marker_size', 45);
    p.parse(varargin{:});

    camera_size_plot_size = p.Results.camera_size_plot_size;
    pc_marker_size = p.Results.pc_marker_size;

    figure;
    hold on;
    % Show cameras poses
    for pose=1:numel(cam_poses)
        % Plot camera
        plotCamera('Location', cam_poses(pose).Translation, 'Orientation', cam_poses(pose).R, ...
            'Size', camera_size_plot_size, 'Color', 'b', 'Opacity', 0);
    end
    % Show point cloud by coloring it based on the pixel color of the first image
    pcshow(point_cloud,'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', pc_marker_size);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    % Rotate and zoom the plot
    camorbit(0, -30);
    hold off;