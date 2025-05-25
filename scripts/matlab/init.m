% Adding appropriate paths:
current_dir = pwd;
functions_dir = sprintf('%s\\matlab_functions', current_dir); 
data_dir = sprintf('%s\\matlab_data', current_dir); 
addpath(functions_dir); % Adding functions directory path allows to call them directly.
addpath(data_dir);

% Load the desired data:
load('merged_robot_data_k1200.0_s-8.5.mat');

% Introduce needed parameters:
angle_shift = 8.5;
Ts = 0.001;
Torque_max = 0.2943;