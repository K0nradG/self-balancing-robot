current_dir = pwd;
functions_dir = sprintf('%s\\matlab_functions', current_dir); 
data_dir = sprintf('%s\\matlab_data', current_dir); 

pid_tuned_and_hand_fixed = load('pid_k886_i10000_d1.mat');
pid_hand_tuned =  load('pid_k1200_i1000_d0.1.mat');

time = (0:length(pid_tuned_and_hand_fixed.theta) - 1) * Ts;
offset = 7.4485; % degrees

figure;
plot(time, pid_tuned_and_hand_fixed.theta + deg2rad(offset))
hold on
plot(time, pid_hand_tuned.theta + deg2rad(offset))
grid on
xlabel("Time [s]")
ylabel("Theta [rad]")
legend("PID tuner with hand fixes", "PID selected manually")
hold off

figure;
plot(time, pid_tuned_and_hand_fixed.pwm)
hold on
plot(time, pid_hand_tuned.pwm)
grid on
xlabel("Time [s]")
ylabel("Control value [%]")
legend("PID tuner with hand fixes", "PID selected manually")
hold off

mean_error_pid_tuned_and_hand_fixed = sum(abs(0 - pid_tuned_and_hand_fixed.theta + deg2rad(offset))) / length(pid_tuned_and_hand_fixed.theta)
mean_error_pid_hand_tuned = sum(abs(0 - pid_hand_tuned.theta + deg2rad(offset))) / length(pid_hand_tuned.theta)