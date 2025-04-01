# Environment Setup

This project is based on **nRF SDK v2.8.0** and **toolchain v2.8.0**.

## 1. Install Required Extensions

To set up the development environment, install the **nRF Connect for VS Code** extension.

🔹 **Important:** Ensure the extension version is **2024.11.22**.

## 2. Install Toolchain and SDK

1. Open the **nRF Connect for VS Code** extension.
2. Install the **toolchain** (v2.8.0).
3. Install **nRF SDK** (v2.8.0).



## 3. Clone the Repository

Once the toolchain and SDK are installed, clone this repository into the following directory:

- `ncs/v2.8.0`



## 4. Switch to the Appropriate West Manifest

To configure the workspace:

1. Open the **nRF Connect for VS Code** extension.
2. Click **Manage West Workspace**.
3. Select **Switch West Manifest Repository**.
4. Search for **self-balancing-robot (self-balancing-robot/west.yaml)** and select it.
5. A window will appear—click **West Update**.

After completing these steps, your environment is ready.

# Building and running

Navigate to the `self-balancing-robot/app` directory and run the following command:

    west build -b nrf52840dongle_nrf52840 -p

## 2. Prepare for Flashing

After building, locate the `merged.hex` file, which is generated in:

- `build/merged.hex`

Copy this file and place it in the **nRF Programmer App**.



## 3. Flash the Firmware

1. Plug in the **nRF52840 Dongle**.
2. Switch it to **Open Bootloader Mode** (this enables DFU over USB).
3. Click the **Write** button in the nRF Programmer App.



**note**

you can switch nrf52840dongle to Open Bootloader Mode by pressing appropriate button - it is a button set at an angle of 90 degrees so as not to press it by accident.

after this nrf52840dongle bootloader is ready for an DFU, so it means it is ready to receive the new image over USB (which we are sending using nrf programmer app) and then swap the new image with the old one.

# Excessive compile errors
In order for `clangd` to work properly a `compile_commands.json` file must be specified as a compilation database. WEST build already creates this file which is inside `app/build/app` directory. 

In order to obtain the `compile_commands.json` run the `get_compile_commands.py` script. Do so when the compilation errors appear again - to update the compilation database after build that caused the problems again.

# The purpose of the project is to build a self balancing robot.

## DONE
- PCB design,
- setup software tools and coding standards,
- agreed on basic software scheme.
  
## TODO:
- investigate PID output/error while positive and negative angle
- tune  PID
- tune alpha in compl filter (IMU)
- introduce low pass filter in regulator.
- measure as much as posiible components before identification.
- model -> https://medium.com/geekculture/dynamics-modelling-and-simulation-of-self-balancing-robot-in-c-d32a3b835bbf



