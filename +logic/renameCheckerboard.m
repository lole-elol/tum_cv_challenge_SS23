function renameCheckerboard(folderName) 
    % RENAMECHECKERBOARD Rename all files in foldername to have the format checkerboard_#.jpg
    % This function is an utility function for the checkerboard calibration
    % process.
    % Inputs:
    %   folderName - The name of the folder containing the images to be renamed
    % Outputs:
    %   None
    
    % Get all files in folder
    files = dir(folderName);
    % Get all files that are not directories
    files = files(~[files.isdir]);
    % Get all files that are not hidden
    files = files(~startsWith({files.name}, '.'));

    % Loop through all files
    for i = 1:length(files)
        % Get the current file
        file = files(i);
        % Get the current file name
        fileName = file.name;
        % Get the current file path
        filePath = fullfile(file.folder, fileName);
        % Get the new file name
        newFileName = sprintf('checkerboard_%d.jpg', i);
        % Get the new file path
        newFilePath = fullfile(file.folder, newFileName);
        % Rename the file
        movefile(filePath, newFilePath);
    end

end