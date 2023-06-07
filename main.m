
% Clear all variables and old windows
close all force
clear all

%% Initialize the environment
% Initialize GUI
global envHandle;
envHandle = env.createGUI();
data = guidata(envHandle.UIFigure);
