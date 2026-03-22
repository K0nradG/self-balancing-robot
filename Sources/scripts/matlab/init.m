% Copyright 2026 Filip Dymczyk and Konrad Grucel

% Adding appropriate paths:
current_dir = pwd;
functions_dir = sprintf('%s\\matlab_functions', current_dir); 
data_dir = sprintf('%s\\matlab_data', current_dir); 
addpath(functions_dir); % Adding functions directory path allows to call them directly.
addpath(data_dir);

% Introduce needed parameters:
angle_shift = 8.5; % - for identification scripts.
Ts = 0.001; % [s]
Torque_max = 0.2943; % [Nm]
offset = 7.4485; % [deg] - for filter and pid testing scripts (setpoint)

max_angle_deviation = deg2rad(2.0); % [rad]
max_angular_velocity_deviation = 0.01; % [rad / s]

% LQR weight matrices:
Q = diag([1 / max_angle_deviation^2, 1 / max_angular_velocity_deviation^2]);
R = 1 / Torque_max^2;