% Plot the point cloud unscaled as  well as the points of the door frame
%figure
plotting.plotPointCloud(denoisedPointCloud, camPoses, cameraSizePlotSize=0.2, pcMarkerSize=20, usePcViewer=true, pcViewerMarkerSize=5)
% plotting.plotPointCloud(denoisedPointCloud2, camPoses, cameraSizePlotSize=0.2, pcMarkerSize=20, usePcViewer=true, pcViewerMarkerSize=5)

% xlabel('X [Unknown]');
% ylabel('Y [Unknown]');
% zlabel('Z [Unknown]');
% hold off