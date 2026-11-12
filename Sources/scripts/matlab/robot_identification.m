% Copyright 2026 Filip Dymczyk and Konrad Grucel

clc; clearvars; close all;

current_dir = pwd;
functions_dir = sprintf('%s\\matlab_functions', current_dir); 
data_dir = sprintf('%s\\matlab_data', current_dir); 
addpath(functions_dir); 
addpath(data_dir);

filename = 'robot_data.csv'; 
data = readtable(filename);

% Trim the first N samples to remove startup glitch
skip_samples = 10; 
data = data(skip_samples:end, :);

Ts = 0.002; % Sampling time: 2 ms
t_data = (0:(height(data)-1)) * Ts; 

% Preprocess measurements with smoothing
p_raw = data.dm;                        
p_meas = smoothdata(p_raw, 'sgolay', 15);        

% Convert angle 'ab' to radians and shift initial hanging position to 0
theta_raw = deg2rad(data.ab);           
theta_meas = theta_raw - theta_raw(1);   

% Combine left and right wheel speeds and PWMs
wheel_speed_avg = (data.s0 + data.s1) / 2; 
pwm_avg = (data.pwm0 + data.pwm1) / 2;     

battery_voltage = 7.4;
u = battery_voltage * (pwm_avg / 100); 

% Compute velocities via numerical differentiation
p_dot_meas = gradient(p_meas, Ts);       
theta_dot_meas = gradient(theta_meas, Ts); 
xr = [p_meas, p_dot_meas, theta_meas, theta_dot_meas];

% Initial state vector
x0 = xr(1, :)'; 

% Initial guess
initial_guess = [0.5, 0.05, 0.015, ...
                 0.1, 0.01, 21.3];

% Parameter bounds
lb = [0.3,  0.01,  0.0001,  0.00001, 0.000001,  4.4];
ub = [3.5,  0.5,  0.1,  1, 0.1, 500];

% Optimization options
options = optimoptions('lsqnonlin', ...
    'Display', 'iter', ...
    'MaxIterations', 500, ...
    'MaxFunctionEvaluations', 10000, ...
    'FiniteDifferenceType', 'central', ...
    'FunctionTolerance', 1e-9, ...
    'ScaleProblem', 'jacobian');

best_params = lsqnonlin( ...
    @(params) objective_function(params, x0, u, Ts, xr), ...
    initial_guess, lb, ub, options);

% 3. Simulate with Optimized Parameters
[t_sim, x_sim] = rk4(x0, u, Ts, best_params);
t_sim_trimmed = t_sim(1:end-1);
x_sim_trimmed = x_sim(1:end-1, :);

% 4. Plot Results
figure('Position', [100, 100, 1000, 800]);
subplot(2,2,1);
plot(t_data, xr(:,1), 'b', t_sim_trimmed, x_sim_trimmed(:,1), 'r--');
title('Robot Position [m]');
legend('Real', 'Sim'); grid on;

subplot(2,2,2);
plot(t_data, xr(:,2), 'b', t_sim_trimmed, x_sim_trimmed(:,2), 'r--');
title('Linear Velocity [m/s]');
legend('Real', 'Sim'); grid on;

subplot(2,2,3);
plot(t_data, rad2deg(xr(:,3)), 'b', t_sim_trimmed, rad2deg(x_sim_trimmed(:,3)), 'r--');
title('Balance Angle Deviation \theta [deg]');
legend('Real', 'Sim'); grid on;

subplot(2,2,4);
plot(t_data, rad2deg(xr(:,4)), 'b', t_sim_trimmed, rad2deg(x_sim_trimmed(:,4)), 'r--');
title('Angular Velocity \dot{\theta} [deg/s]');
legend('Real', 'Sim'); grid on;

disp('Best params');
disp(best_params);