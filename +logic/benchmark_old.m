function benchmark_old(image_folder, config_file, varargin)
% BENCHMARK Run the benchmark for the given image folder and all configs DEPRECATED
% wich are specified in the config file
%   image_folder: path to folder with images
%   config_file: fpath to file with configs

configs = read_config(config_file, "configs");
camera_params= load("test/params/camera_params.mat").camera_params;
% Get all files that are not directories
files = dir(image_folder);
files = files(~[files.isdir]);
results = "empty";


% Run for each config
for i =1:length(configs)
    config = configs(i);

    % Run for pairwise images
    for f = 1:2:length(files)

        folder = files(f).folder;
        file = "image" + int2str(f) + ".jpg";
        next_file = "image" + int2str(f+1) + ".jpg";

        I1 = imread(fullfile(folder, file));
        I2 = imread(fullfile(folder, next_file));
        field ="config_"+int2str(i) + "_image" + int2str(f) + "_" + int2str(f+1);

        value =logic.reconstruct3D(I1, I2, camera_params, config);
        % just print to console for now
        % i found no easy way to append them to a struct
        field, value
    end

end

