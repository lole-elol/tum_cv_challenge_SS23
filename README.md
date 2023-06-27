# TUM Computer Vision Challenge SS2023

## Project structure

```
+gui/
  createGui.m
  +cb/
+logic/
docs/
bin/
README.txt
main.m
```

The program started using `main.m`. All GUI related files are located in the subfolder `+gui/`. `+cb/` contains all callback functions which are need for GUI interactions like "Button Press". The program logic is located in `+logic/`.

`bin/` includes utility scripts for example for working with the demo data.


## Coding conventions

### Code Documentation

Each function should include a comment explaining what it does and defining the input an output parameters. The comment should look as follows:

```matlab
% FUNCTIONNAME - short description of your function
%   Optional longer description of your function
% Inputs:
%   inputParam: Description of the input parameter
%   optionalParam = "default value": Description of the optional input parameter
% Outputs:
%   outputParam: Description of the output parameter
```

### Variable conventions
- Use `camelCase` for variables.

### Function conventions

- Use `camelCase` for functions.
- No indention inside a function. Only indent if/for/while statements. 
- Use `Name=Value` for function arguments instead of `"Name", "Value"`. E.g.:
```matlab
function outputVar = helloWorld(inputVar)
% HELLOWORLD displays hello world and checks if 1 is actually 1
% Inputs:
%   inputVar: some input that is irrelebant
% Outputs:
%   outputVar: some output that is always 1
disp("hello world!");
if 1 ~= 1
  disp("Everything is a lie");
  status = callInCaseOfEmergency(destroyWorld=true)
end
outputVar = 1;
end
```

## Toolboxes:
(command: `ver`)
MATLAB                                                Version 9.14        (R2023a)
Computer Vision Toolbox                               Version 10.4        (R2023a)
Image Processing Toolbox                              Version 11.7        (R2023a)
Lidar Toolbox                                         Version 2.3         (R2023a)
Signal Processing Toolbox                             Version 9.2         (R2023a)
Statistics and Machine Learning Toolbox               Version 12.5        (R2023a)
