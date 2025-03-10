## Inverted pendulum model

Model of **an inverted pendulum on a cart** (self-balancing robot) can be described by two differential equations. However, in our case we are only concerned about the model of the **inverted pendulum** itself - we are not controlling the robot position, but only trying to **balance** it:

$$
(I + m\ell^2) \ddot{\theta} = m\ell \ddot{x} \cos{\theta} + b \dot{\theta} + m g \ell \sin{\theta}.
$$

#### Where:
- $ I $ — moment of inertia of the pendulum about its center of mass [$\text{kg} \cdot \text{m}^2$],
- $ m $ — mass of the pendulum [$\text{kg}$],
- $ \ell $ — distance from the pivot point to the center of mass of the pendulum [$\text{m}$],    
- $ b $ — damping coefficient [$\text{N} \cdot \text{m} \cdot \text{s} / \text{rad}$],  
- $ g $ — gravitational acceleration [$\text{m/s}^2$].   

In our case, the center of mass was **very close** to the pivot point. That is why, we can simplify the equations by getting rid of **$ I $**, which after simplifications leads us to:

$$
\ell \ddot{\theta} = \ddot{x}\cos{\theta} + b \dot{\theta} + g \sin{\theta}.
$$

This equation was modeled in `pendulum_model.slx` Simulink file, for some random parameters from `simulation_data.m`, to test the model behavior. 

> **Note:** In order to run the simulation, launch the data file first.

It is also worth mentioning that for our purpose of later identification, simplified equation **won't lead** to any error, since it can be always written as:

$$
a \cdot \ddot{\theta} = b \cdot \ddot{x}\cos{\theta} + c \cdot \dot{\theta} + d \cdot \sin{\theta}.
$$

Where **a**, **b**, **c**, **d** are unknown coefficients to be identified.

Also, for **small angles** we can approximate:

$$
\sin(\theta) \approx \theta,\quad \cos(\theta) \approx 1.
$$

This leaves us with:

$$
\ell \ddot{\theta} = \ddot{x} + b \dot{\theta} + g \theta.
$$

Dividing by **$\ell$** gives us:

$$
\ddot{\theta} = \frac{\ddot{x}}{\ell} + \frac{b}{\ell} \dot{\theta} + \frac{g}{\ell} \theta.
$$


## Continous transfer fucntion
As mentioned before, we are **not** controlling the robot position. Thus, the **$\ddot{x}$** term is simply an input to our system. In `motor_controller` files we are controlling the motors by applying certain **PWM** level to them. Through that, we produce the input **$\ddot{x}$**, which can be obtained from:

$$
\ddot{x} = \frac{F}{m}, \quad \text{where} \quad F = k \cdot u.
$$

As we can see, the control input **u** (PWM signal) is directly related to the force driving the motors through some constant **k** (in simple linear case). This allows us to derive the **transfer function** for our system (assuming zero initial conditions):

$$
s^2 \Theta(s) = \frac{k}{m\ell} U(s) + \frac{b}{\ell} s \Theta(s) + \frac{g}{\ell} \Theta(s)
$$

Rearranging terms:

$$
\left( s^2 - \frac{b}{\ell} s - \frac{g}{\ell} \right) \Theta(s) = \frac{k}{m\ell} U(s).
$$

Solving for the transfer function:

$$ 
G(s) = \frac{\Theta(s)}{U(s)}.
$$

We get:

$$
G(s) = \frac{\frac{k}{m\ell}}{s^2 - \frac{b}{\ell} s - \frac{g}{\ell}}.
$$

It clearly shows that the open-loop system is unstable.

As we can see, there are 3 variables to be identified:

$$ 
a_1 = \frac{k}{m\ell}, \quad a_2 = \frac{b}{\ell}, \quad a_3 = \frac{g}{l}.
$$

## Discrete Transfer Function

Our work is performed using a **microcontroller**. That is why it is important to include the sampling period **$T_{s}$** in the system. The discrete transfer function can be obtained from the continuous one using the **forward-Euler** method:

$$
\frac{d}{dt} \approx \frac{z - 1}{T_s},
$$

Substituting **$s$** and **$s^2$** with the forward-Euler derivative approximation gives us:

$$
G(z) = \frac{\frac{k T_s^2}{m\ell}}{(z - 1)^2 - \frac{b}{\ell} T_s \cdot (z - 1) - \frac{g}{\ell} T_s^2}.
$$

After rearranging the terms, we obtain:

$$
G(z) = \frac{\frac{k T_s^2}{m\ell}}{z^2 - \left( 2 + \frac{b}{\ell} T_s \right) z + \left( 1 + \frac{b}{\ell} T_s - \frac{g}{\ell} T_s^2 \right)}.
$$

## Identification

For proper control loop tuning, **system identification** will be performed based on data collected from an IMU unit (MPU6050). After obtaining the data with the use of e.g. `receive_serial_data.py` script, we can run `parse_to_mat.py` script to parse the received data into `.mat` format accepted by the identification scripts. `identification.m` script implements continuous system identification based on the data entered and `identification_discrete.m` - discrete system identification with fixed sampling rate used in the control loop. 

> **Note:** In order to perform identification for the inverted pendulum, first the partial stabilization using some custom P/PI controller must be done. The stabilization parameters are then incorporated into the system closed loop transfer function, which then gets identified. Since the parameters of the stabilization regulator will be known, the unknown parameters can be easily calculated from that point.

Additionally, `identified_discrete_model.slx` allows to run the simulation for the obtained transfer function of a discrete inverted pendulum model (first `identification_discrete.m` file needs to be run).