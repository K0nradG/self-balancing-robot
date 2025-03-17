## Identification basics

For proper control loop tuning, **system identification** will be performed based on data collected from an IMU unit (MPU6050). After obtaining the data with the use of e.g. `receive_serial_data.py` script, we can run `parse_to_mat_with_merge.py` script to take the entered data files names, parse and merge them into one `.mat` format accepted by the identification scripts.

## Closed loop identification

In order to perform identification for the inverted pendulum, first the **partial stabilization** using some custom **P/PI** controller must be done. The stabilization parameters are then incorporated into the system **closed loop** transfer function, which then gets identified. Since the parameters of the stabilization regulator will be **known**, the unknown system parameters can be easily calculated from that point.

To remind, model transfer function can be written as:

$$
G(s) = \frac{a_1}{s^2 + a_2 s + a_3}.
$$

## P regulator identification
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

This approach was implemented in the `closed_loop_identification_P.m` script.

# PI Regulator Identification

Using a **PI controller** can provide more **stable** measurements for system identification, but also turbulent enough to capture the **system dynamics**. To derive model parameters from the closed-loop system, we start with the **PI controller** transfer function:

$$
G_{PI}(s) = K_p + \frac{K_i}{s}.
$$

Rewriting it in a standard form:

$$
G_{PI}(s) = \frac{K_p s + K_i}{s}.
$$

The **series connection** of the object and the PI controller results in the following transfer function:

$$
G_{system}(s) = G_{object} * G_{PI} = \frac{a_1(K_p s + K_i)}{s^3 + a_2 s^2 + a_3s}.
$$

Closing the control loop with **negative feedback** leads to:

$$
G_{system}(s) = \frac{a_1(K_p s + K_i)}{s^3 + a_2 s^2 + (a_3 + K_p a_1)s + K_i a_1}.
$$

After identification, the model parameters can be derived similarly to the **P and PID controllers**, by making the substitutions:

$$
G_{system}(s) = \frac{b_1s + b_0}{s^3 + c_2s^2 + c_1s + c_0}.
$$

Where:
- $b_1 = K_p a_1$,
- $b_0 = K_i a_1$,
- $c_2 = a_2$,
- $c_1 = a_3 + b_1$,
- $c_0 = b_0$.

This method was implemented in the `closed_loop_identification_PI.m` script.


## PID regulator identification

Similarly to the **PI controller** we can derive the identification based on the **PID controller**:

$$
G_{PID}(s) = K_p + \frac{K_i}{s} + K_ds.
$$

We can rewrite it as:

$$
G_{PID}(s) = \frac{K_ds^2 + K_ps + K_i}{s}.
$$

**Series** connection of the object and the PID controller will result in the below transfer function:

$$
G_{system}(s) = G_{object} * G_{PID} = \frac{a_1(K_ds^2 + K_ps + K_i)}{s^3 + a_2 s^2 + a_3s}.
$$

Closing the control loop with **negative feedback** finally results in:

$$
G_{system}(s) = G_{object} * G_{PID} = \frac{a_1(K_ds^2 + K_ps + K_i)}{s^3 + (a_2 + K_d \cdot a_1)s^2 + (a_3 + K_p \cdot a_1)s + K_i \cdot a_1}.
$$

Knowing the **PID controller** parameters, after identification we can (similarly to the P controller) derive the model parameters with substitutions:

$$
G_{system}(s) = \frac{b_2s^2 + b_1s + b_0}{s^3 + c_2s^2 + c_1s + c_0}.
$$

Where:
- $b_2 = K_da_1$,
- $b_1 = K_pa_1$,
- $b_0 = K_ia_1$,
- $c_2 = a_2 + b_2$,
- $c_1 = a_3 + b_1$,
- $c_0 = b_0$.

This approach was implemented in the `closed_loop_identification_PID.m` script.

## Least Squares Method

The **Least Squares** method is a mathematical optimization technique used to find the best-fitting solution to a system of equations that may not have an exact solution. It minimizes the sum of the squared differences (or residuals) between the observed values (data points) and the values predicted by the model.

#### Matrix Formulation

In the general case of linear regression or system identification, the model is expressed as:

$$
Y = X \Theta + \epsilon
$$

where:
- $ Y $ is the vector of observed outputs,
- $ X $ is the matrix of inputs (features),
- $ \Theta $ is the vector of model parameters,
- $ \epsilon $ is the error term.

The Least Squares solution for $ \Theta $ is given by:

$$
\Theta = (X^T X)^{-1} X^T Y
$$

$X$ columns contain **inputs** to the system: $\theta$, $\dot{\theta}$ and control signal value (PWM). $Y$ contains the **output** the system is to be fitted to: $\ddot{\theta}$ - calculated by **differentiating** the input $\dot{\theta}$. When differentiating, applying some form of **filtering** e.g. a **Butterworth filter**, could be desired since when calculating the second derivative directly some errors may be encountered that may affect the whole identification process. $\Theta$ will contain the parameters that best satisfy the least squares procedure.

This approach was implemented in the `closed_loop_identification_LS.m` script. It also implements the calculation of model parameters from the identified **closed loop system** with only the **proportional controller**.