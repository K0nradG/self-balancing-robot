## Inverted pendulum

Model of an inverted pendulum on a cart (self-balancing robot) can be described by two differential equations. However, in our case we are only concerned about the model of the inverted pendulum itself - we are not controlling the robot position, but only trying to balance it:

$$
(I + m\ell^2) \ddot{\theta} = m\ell \ddot{x} \cos{\theta} + b \dot{\theta} + m g \ell \sin{\theta}.
$$

#### Where:
- $ I $ — moment of inertia of the pendulum about its center of mass [$\text{kg} \cdot \text{m}^2$],
- $ m $ — mass of the pendulum [$\text{kg}$],
- $ \ell $ — distance from the pivot point to the center of mass of the pendulum [$\text{m}$],    
- $ b $ — damping coefficient [$\text{N} \cdot \text{m} \cdot \text{s} / \text{rad}$],  
- $ g $ — gravitational acceleration [$\text{m/s}^2$].   

In our case, the center of mass was very close to the pivot point. That is why, we can simplify the equations by getting rid of $ I $:

$$
\ell \ddot{\theta} = \ddot{x}\cos{\theta} + b \dot{\theta} + g \sin{\theta}.
$$

This equation was modeled in `pendulum_model.slx` Simulink file, for some random parameters from `simulation_data.m`, to test the model behavior.

It is also worth mentioning that for our purpose of later identification, simplified equation won't lead to any error, since it can be always written as:

$$
a \cdot \ddot{\theta} = b \cdot \ddot{x}\cos{\theta} + c \cdot \dot{\theta} + d \cdot \sin{\theta}.
$$

Where a, b, c, d are unknown coefficients to be identified.

Also, for small angles we can approximate:

$$
\sin(\theta) \approx \theta,\quad \cos(\theta) \approx 1.
$$

Which leaves us with:

$$
\ell \ddot{\theta} = \ddot{x} + b \dot{\theta} + g \theta.
$$

Dividing by $\ell$ gives us:

$$
\ddot{\theta} = \frac{\ddot{x}}{\ell} + \frac{b}{\ell} \dot{\theta} + \frac{g}{\ell} \theta.
$$

As mentioned before, we are not controlling the robot position. Thus, the $\ddot{x}$ term is simply an input to our system. This allows us to derive the transfer function for our system (assuming zero initial conditions):

$$
s^2 \Theta(s) = \frac{1}{\ell} \ddot{X}(s) + \frac{b}{\ell} s \Theta(s) + \frac{g}{\ell} \Theta(s)
$$

Rearranging terms:

$$
\left( s^2 - \frac{b}{\ell} s - \frac{g}{\ell} \right) \Theta(s) = \frac{1}{\ell} \ddot{X}(s).
$$

Solving for the transfer function:
$$ 
G(s) = \frac{\Theta(s)}{\ddot{X}(s)}.
$$

We get:

$$
G(s) = \frac{\frac{1}{\ell}}{s^2 - \frac{b}{\ell} s - \frac{g}{\ell}} .
$$

It clearly shows that the open loop system is unstable.

For proper control loop tuning, system identification will be performed based on data collected from an IMU unit (MPU6050). Also, our work is performed with use of a micro controller. That is why, it is important to include the sampling period $T_{s}$ in the system.

`identification.m` script implements continuous system identification based on the data entered and `identification_discrete.m` - discrete system identification with fixed sampling rate used in the control loop.