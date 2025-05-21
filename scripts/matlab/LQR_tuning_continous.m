% Weight matrices:
Q = diag([(1 / rad2deg(0.5)^2), (1 / 0.05)^2]);
R = 1 / (0.2943)^2;

K = lqr(ss_continous.A, ss_continous.B, Q, R);

disp('Kalman Gain:');
disp(K)

eig_vals = eig(ss_continous.A - ss_continous.B * K);
isstable(ss(ss_continous.A - ss_continous.B * K, ss_continous.B, ss_continous.C, ss_continous.D))