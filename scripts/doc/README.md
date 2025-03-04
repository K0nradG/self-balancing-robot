# Python scripts overview

## `Clangd` issues fix

In order for `clangd` to work properly, `compile_commands.json` must be acquired from `CMake` build. The `get_compile_commands.py` takes the generated file from build directory and formats it by getting rid of flags not recognized by `clang`.

Whenever compilation warnings/errors occur, try running `get_compile_commands.py` script and reloading the VS Code window (ctrl + shift + p). This should fix the issues.

## General Serial Data Logging

- plug in NRF Dongle via USB cable,
- open `receive_serial_data.py` - modify `COM` port name (specific to Windows and Linux),
- run `receive_serial_data.py` script,
- press button on Dongle to start receiving through the script,
- once the sending is finished, data will be logged to `data/minicom.txt` file (can be changed in the script).

## Model Identification

To identify the model, first run the **`receive_serial_data.py`** script to gather data from the Dongle. Then, run the **`estimate_model.py`** script.

The result will be the estimated model parameters $ \alpha_1 $, $ \alpha_2 $, and $ \alpha_3 $, where:

- $ \alpha_1 = -\frac{b}{I} $ (damping coefficient over moment of inertia),
- $ \alpha_2 = \frac{m \cdot g \cdot l}{I} $ (gravitational term, based on the pendulum's mass, length, and gravitational acceleration),
- $ \alpha_3 = \frac{1}{I} $ (inverse of moment of inertia, affecting the control input).

These parameters are used in the state-space model of the inverted pendulum:

$$
\dot{x_1} = x_2
$$

$$
\dot{x_2} = \alpha_1 \cdot x_2 + \alpha_2 \cdot \sin(x_1) + \alpha_3 \cdot u
$$

Where:
- $ x_1 = \theta $ (the angle of deviation),
- $ x_2 = \dot{\theta} $ (the angular velocity),
- $ u $ (control input, the torque applied by the motor).

Using these parameters, you can further analyze or design a controller for the balancing robot.

