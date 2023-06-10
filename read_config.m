

function config = read_config(config_file, config_type)
% Reads a config.json  and returns a struct with the config values

config = jsondecode(fileread(config_file)).(config_type)

end
