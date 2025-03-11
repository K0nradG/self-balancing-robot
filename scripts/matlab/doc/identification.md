## Identification basics

For proper control loop tuning, **system identification** will be performed based on data collected from an IMU unit (MPU6050). After obtaining the data with the use of e.g. `receive_serial_data.py` script, we can run `parse_to_mat.py` script to parse the received data into `.mat` format accepted by the identification scripts. `identification.m` script implements continuous system identification based on the data entered and `identification_discrete.m` - discrete system identification with fixed sampling rate used in the control loop. 

## Closed loop identification

In order to perform identification for the inverted pendulum, first the **partial stabilization** using some custom **P/PI** controller must be done. The stabilization parameters are then incorporated into the system **closed loop** transfer function, which then gets identified. Since the parameters of the stabilization regulator will be **known**, the unknown system parameters can be easily calculated from that point.

To remind, model transfer function can be written as:

$$
G(s) = \frac{a_1}{s^2 + a_2 s + a_3}.
$$

Assuming the use of **Proportional** controller with its gain given by **$K_p$**, closed loop transfer function of continuous model would be given by:

$$
G_{closed}(s) = \frac{K_p \cdot a_1}{s^2 + a_2 s + a3 + K_p \cdot a_1}.
$$

Nominator and last denominators terms can be written as $b_0$, $b_1$ respectively for simplification. Then, the closed loop transfer function would be given by:

$$
G_{closed}(s) = \frac{b_0}{s^2 + a_2 s + b_1}.
$$

Where:
- $b_0 = K_p \cdot a_1$.
- $b_1 = a_3 + b_0$.

Having identified the closed loop transfer function and knowing **Kp** , we can easily calculate the inverted pendulum model parameters.

Knowing the model continuos transfer function we can change it to state-space representation (using **ts2ss()** function in MATLAB) to obtain the A, B, C and D matrices of the system. Then we can calculate the discrete model using **forward-Euler** approximation:

$$
A_d = I - T_s \cdot A, \quad B_d = T_s \cdot B, \quad C_d = C, \quad D_d = D.
$$

The above mentioned information is implemented in the `closed_loop_identification.m` script.

Discrete state-space or transfer function representation of the object can be used directly to optimize PID or even LQR controller.