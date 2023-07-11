# TUM Computer Vision Challenge SS2023

![Poster](poster.svg)
## Usage

The GUI can be started by running `main.m`. This will open a window where you can choose images of a room. The program will then reconstruct a 3D Model of this room.

More information about the user interface can be found in [+gui/README.md](+gui/README.md)

## Project structure

```
+benchmark/
+gui/
  createGui.m
  +cb/
+logic/
  +pointcloud/
  +reconstruct3D/
+plotting/
+util/
config/
README.txt
main.m
pipeline.m
```

The program started using `main.m`. All GUI related files are located in the subfolder `+gui/`. The program logic is located in `+logic/`.

`+util/` includes utility scripts for example for working with the demo data. The code in `+plotting/` is for plotting related functions (usually for debugging)

`+benchmark/` contains a complete toolset for benchmarking 3d reconstruction and model detection. More information can be found in [+benchmark/README.md](+benchmark/README.md)

## Toolboxes:
(command: `ver`)
MATLAB                                                Version 9.14        (R2023a)
Computer Vision Toolbox                               Version 10.4        (R2023a)
Image Processing Toolbox                              Version 11.7        (R2023a)
Lidar Toolbox                                         Version 2.3         (R2023a)
Medical Imaging Toolbox                               Version 1.1         (R2023a)
Signal Processing Toolbox                             Version 9.2         (R2023a)
Statistics and Machine Learning Toolbox               Version 12.5        (R2023a)

## Data

Data to large to be transmitted/handled by git is accessible over [lrz.de](lrz.de).

- [Test Data](https://syncandshare.lrz.de/getlink/fiW28ckD2bDDu6u2jVv7m7/test): Demo Images and Pointclouds. Download this data into the root of the project
- [Benchmark Data](https://syncandshare.lrz.de/getlink/fi8T7n2HSZSyuuPH63AWik/): Results from Benchmarks. Download this into `+benchmark/demoData/` to analyse the results

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

### Naming conventions

#### Variables
- Use `camelCase` for variables.

#### Functions

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
