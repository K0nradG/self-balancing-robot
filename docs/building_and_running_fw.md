# Building and flashing

## 1. Building 

Navigate to the `Sources/applications/app` directory and run the following command:

    west build -b nrf52840dongle_nrf52840 -p

or use the `build_dk_sw.py` script inside the `nRF Connect` terminal - it navigates to the necessary path, builds the application and retrieves the `compile_commands.json` used by `clangd`.

## 2. DFU over BLE

After building, connect with **RPI** web-server (connection to its network) and go to the dedicated site. There, it is possible to upload the `dfu_application.zip` generated inside `Sources/applications/app/build` directory and launch the procedure. After it is finished, the robot is ready to be controlled with a **new firmware version**.