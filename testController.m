clear; close all; clc;
% Add the helper functions to the Matlab search path
addpath('./helperFunctions/');

constants;
ctrl = controller;
dt = 1/250;

max_step = 100;

for i = 0:(max_step - 1)
  
end