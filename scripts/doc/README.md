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

To identify the model, first run `receive_serial_data.py` to get the data from Dongle and then run `estimate_model.py` script. 

The result will be the $\alpha$ and $\beta$ model parameters, where:

- $\alpha$ = (m * g * l) / I,
- $\beta$ = 1 / I.
