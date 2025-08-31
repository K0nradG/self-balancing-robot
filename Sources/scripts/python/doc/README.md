# Python scripts overview

## Clangd issues fix

In order for `clangd` to work properly, `compile_commands.json` must be acquired from `CMake` build. The `get_compile_commands.py` takes the generated file from build directory and formats it by getting rid of flags not recognized by `clang`.

Whenever compilation warnings/errors occur, try running `get_compile_commands.py` script and reloading the VS Code window (ctrl + shift + p). This should fix the issues.

## General Serial Data Logging

- plug in NRF Dongle via USB cable,
- open `receive_serial_data.py` - modify `COM` port name (specific to Windows and Linux),
- run `receive_serial_data.py` script,
- press button on Dongle to start receiving through the script,
- once the sending is finished, data will be logged to a specified location.

## Parsing to valid .mat file

MATLAB scripts created to **identify object model** based on the data gathered from Dongle accept only a **specific** data format. After receiving the serial data (e.g. running `receive_serial_data.py`) run `parse_to_mat_with_merge.py`. The script will take the specified data and combine them into one `.mat` format accepted by the `.m` scripts. After that, MATLAB scripts can be run to perform identification and/or regulators tuning.