## Identification basics

For proper control loop tuning, **system identification** will be performed based on data collected from an IMU unit (MPU6050). After obtaining the data with the use of e.g. `receive_serial_data.py` script, we can run `parse_to_mat.py` script to parse the received data into `.mat` format accepted by the identification scripts. `identification.m` script implements continuous system identification based on the data entered and `identification_discrete.m` - discrete system identification with fixed sampling rate used in the control loop. 

## Closed loop identification

In order to perform identification for the inverted pendulum, first the **partial stabilization** using some custom **P/PI** controller must be done. The stabilization parameters are then incorporated into the system **closed loop** transfer function, which then gets identified. Since the parameters of the stabilization regulator will be **known**, the unknown system parameters can be easily calculated from that point.

Assuming the use of **Proportional** controller with its gain given by **$K_p$**, closed loop transfer function of continuous model would be given by:

$$
G_{closed}(s) = \frac{K_p \cdot \frac{k}{m\ell}}{s^2 + \frac{b}{\ell} s - \left( \frac{g}{\ell} - K_p \cdot \frac{k}{m\ell} \right)}.
$$

To simplify we can use $a_1, a_2$, and $a_3$ to substitute for our unknown terms:

$$
G_{closed}(s) = \frac{K_p \cdot a_1}{s^2 + a_2 s + a_3}.
$$

Where:
- $a_1 = \frac{k}{m\ell}$,
- $a_2 = \frac{b}{\ell}$,
- $a_3 = -\frac{g}{l} + K_p * a_1$.

Knowing **Kp** and all of the unknowns, we can easily calculate the inverted pendulum model parameters.

Knowing the model continuos transfer function we can change it to state-space representation (using **ts2ss()** function in MATLAB) to obtain the A, B, C and D matrices of the system. Then we can calculate the discrete model using **forward-Euler** approximation:

$$
A_d = I - T_s \cdot A, \quad B_d = T_s \cdot B, \quad C_d = C, \quad D_d = D.
$$

The above mentioned information is implemented in the `closed_loop_identification.m` script.

Discrete state-space or transfer function representation of the object can be used directly to optimize PID or even LQR controller.