% Copyright 2026 Filip Dymczyk and Konrad Grucel

clc; clearvars; close
init;

pid_filter = load('pid_filter_0.1.mat');
pid_no_filter =  load('pid_no_filter.mat');

time = (0:length(pid_filter.theta) - 1) * Ts;

figure;
plot(time, pid_filter.theta + deg2rad(offset))
hold on
plot(time, pid_no_filter.theta + deg2rad(offset))
grid on
xlabel("Time [s]")
ylabel("Theta [rad]")
legend("PID with filter", "PID with no filter")
hold off

figure;
plot(time, pid_filter.pwm)
hold on
plot(time, pid_no_filter.pwm)
grid on
xlabel("Time [s]")
ylabel("Control value [%]")
legend("PID with filter", "PID with no filter")
hold off

mean_error_pid_filter = mean(abs(0 - pid_filter.theta + deg2rad(offset)))
mean_error_pid_no_filter = mean(abs(0 - pid_no_filter.theta + deg2rad(offset)))