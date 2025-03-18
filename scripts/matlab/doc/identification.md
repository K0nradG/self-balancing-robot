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

This approach was implemented in the `closed_loop_identification_P.m` script using **System Identification Toolbox** in MATLAB.

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

This method was implemented in the `closed_loop_identification_PI.m` script using **System Identification Toolbox** in MATLAB.


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

This approach was implemented in the `closed_loop_identification_PID.m` script using **System Identification Toolbox** in MATLAB.

> **Note!** Apart from the first method of identification based on closed loop (with only proportional controller), the results were rather poor. Is is believed that the reason for that was too much stability, which caused the controller input to cover the object real dynamics. This, in turn, made it really hard to estimate the system true behavior.

**That is why, for the other approaches only the proportional controller closed loop identification method was used.**

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

This approach was implemented in the `closed_loop_identification_LS.m` script. It implements the calculation of model parameters from the identified **closed loop system** with only the **proportional controller**.

## Optimization Approach

The problem of determining model parameters can be formulated as **finding a curve that best fits** the given data. The previous approach based on the **Least Squares** (LS) method had a significant drawback — the necessity to compute $\ddot{\theta}$ directly by differentiating $\dot{\theta}$. This process introduced a **high level of noise**, which had to be filtered before performing identification.

To overcome this issue, the following approach has been proposed:

1. **Estimate** the unknown model parameters.
2. **Substitute** them into the system equation.
3. **Solve** the system of differential equations, e.g., using the **Runge-Kutta** 4th order (RK4) method.
4. Create a **cost function**, such as the **sum of squared errors** between the solution and the input data.
5. **Minimize** the objective function to improve model fitting.
6. Obtain **new estimates** of the parameters.
7. **Reinsert** the parameters into the equations and **repeat** the entire process until a specified level of error between the curves is achieved.

### RK4 method for solving differential equations

**The Runge-Kutta 4th order (RK4)** method is one of the most commonly used numerical algorithms for solving **ordinary differential equations** (ODEs).

### RK4 Algorithm
For a differential equation of the form:

$$
\frac{dy}{dt} = f(t, y), \quad y(t_0) = y_0.
$$

the **RK4** method computes successive values of $y$ using the formula:

$$
 y_{n+1} = y_n + \frac{1}{6} (k_1 + 2k_2 + 2k_3 + k_4) 
$$

where the coefficients $ k_1, k_2, k_3, k_4 $ are calculated as:

$$
 k_1 = h f(t_n, y_n) \\[1em]
 k_2 = h f(t_n + \frac{h}{2}, y_n + \frac{k_1}{2}) \\[1em]
 k_3 = h f(t_n + \frac{h}{2}, y_n + \frac{k_2}{2}) \\[1em]
 k_4 = h f(t_n + h, y_n + k_3) 
$$

where $h$ is the time step.

This approach was implemented in the `closed_loop_identification_RK4.m` script, where RK4 method of constant solving the differential equations was combined with MATLAB `fminsearch` function. Functions necessary for the script were provided in the `matlab_functions` directory.

The result of this method resulted in **reliable** results, which allowed for proper **PID controller tuning**.