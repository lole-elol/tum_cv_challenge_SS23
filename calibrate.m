% This file calibrates the camera once and saves it to a file for later use
camera_params = logic.calibrateCamera("test/checkerboard", 15);
save("test/params/camera_params.mat", "camera_params");