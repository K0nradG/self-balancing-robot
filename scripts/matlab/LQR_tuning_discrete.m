% Weight matrices:
Q = diag([100, 1]);
R = 1;

K = dlqr(ss_object_discrete.A, ss_object_discrete.B, Q, R);

disp('Kalman Gain:');
disp(K)

eig_vals = eig(ss_object_discrete.A - ss_object_discrete.B * K);

if all(abs(eig_vals)< 1) 
    disp('System is stable after closing')
else
    disp('System still unstable')
end