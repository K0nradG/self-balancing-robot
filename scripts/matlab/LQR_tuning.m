% Weight matrices:
Q = diag([10, 1]);
R = 1; 

K = dlqr(A_d, B_d, Q, R);

disp('Kalman Gain:');
disp(K);