# Building and flashing

[Project overview](../README.md) |
[Environment setup](environment_setup.md) |
**Building and flashing**

This guide describes how to build the robot firmware with the nRF Connect
extension for Visual Studio Code or directly from a terminal. It also covers
the initial SWD flash and subsequent firmware updates over Bluetooth LE.

Complete the [environment setup](environment_setup.md) before continuing.

## Application and board

| Setting | Value |
| --- | --- |
| Application directory | `Sources/applications/app` |
| Board target | `nrf52840dongle_nrf52840` |
| nRF Connect SDK | v2.8.0 |
| Toolchain | v2.8.0 |
| Bootloader | MCUboot |

## Option 1: Build with nRF Connect for VS Code

1. Open the west workspace in Visual Studio Code.
2. Open the **nRF Connect** extension from the activity bar.
3. In the **Applications** view, select **Add an existing application**.
4. Select:

   ```text
   self-balancing-robot/Sources/applications/app
   ```

5. Select **Add build configuration** for the application.
6. Configure the build:
   - **Board:** `nrf52840dongle_nrf52840`
   - **SDK:** nRF Connect SDK v2.8.0
   - **Toolchain:** v2.8.0
   - **Build directory:** `build`
   - **Pristine build:** enabled for the first build
7. Select **Build configuration**.

The build output and available actions are displayed in the nRF Connect
extension. Later builds can use **Build** without recreating the
configuration. Use **Pristine Build** after changing the board, devicetree
overlays, Kconfig files, or SDK version.

## Option 2: Build from the terminal

Open a terminal with the nRF Connect SDK v2.8.0 environment active. You can
open one from nRF Connect for Desktop/Toolchain Manager, from the nRF Connect
extension, or configure the SDK environment manually.

From the repository root:

```bash
cd Sources/applications/app
west build -b nrf52840dongle_nrf52840 -p always
```

`-p always` performs a pristine build. For subsequent builds, run:

```bash
west build
```

To rebuild after changing CMake, Kconfig, or devicetree configuration:

```bash
west build -p always
```

To inspect or change Kconfig options interactively:

```bash
west build -t menuconfig
```

### Build helper script

The repository also provides a helper script:

```bash
python3 build_dongle_sw.py
```

Run it from the repository root in an nRF Connect terminal. The script:

1. formats files changed in the working tree,
2. builds `Sources/applications/app`,
3. retrieves `compile_commands.json` for `clangd`.

> [!IMPORTANT]
> The robot firmware helper is `build_dongle_sw.py`. The
> `Sources/scripts/python/build_dk_sw.py` script builds the separate nRF7002 DK
> controller application and must not be used for the robot firmware.

## Build artifacts

A successful build creates the `Sources/applications/app/build` directory.

| Artifact | Purpose |
| --- | --- |
| `build/merged.hex` | Complete image for the initial SWD flash, including MCUboot |
| `build/dfu_application.zip` | Signed application update package for DFU over BLE |

The DFU package contains the signed MCUboot application image. Do not use an
unsigned application image for DFU.

## Initial flash over SWD

The nRF52840 Dongle does not have an onboard debugger. The first complete
firmware installation, including MCUboot, requires an external SWD debugger.
The project has been tested using an nRF7002 DK as the debugger.

1. Connect the external debugger to the nRF52840 Dongle SWD signals.
2. Build the application.
3. From `Sources/applications/app`, run:

   ```bash
   west flash --erase --skip-rebuild
   ```

The same operation can be started from the repository root with:

```bash
python3 Sources/scripts/python/flash_dongle.py
```

`--erase` removes the previous flash contents. Use it for the initial
installation or when the partition layout changes. It also removes settings
stored in flash.

The nRF Connect extension can perform the equivalent operation through the
**Flash** action after a build configuration has been created and a supported
debug probe is connected.

## Firmware update over Bluetooth LE

After the initial SWD installation, application updates can be installed over
Bluetooth LE without opening the robot.

### Desktop application

1. Build the new firmware.
2. Start the desktop control application:

   ```bash
   python3 run_controller_app.py
   ```

3. Connect to `SELF_BALANCING_ROBOT`.
4. Open the DFU view and select:

   ```text
   Sources/applications/app/build/dfu_application.zip
   ```

5. Start the update and keep the robot powered until it reboots and
   reconnects.

The desktop DFU workflow uses `smpmgr`. A Windows binary is included in
`3rdParty/smpmgr`; on Linux, `smpmgr` must be available on `PATH`.

### Command-line DFU

The same update can be started from the repository root:

```bash
python3 dfu_ble_dongle.py
```

By default, the script:

- connects to the BLE device named `SELF_BALANCING_ROBOT`,
- extracts `app.signed.bin` from `dfu_application.zip`,
- uploads and tests the image,
- resets the robot,
- reconnects and confirms the new image.

Use `--help` to list options such as a different BLE address or image:

```bash
python3 dfu_ble_dongle.py --help
```

### Raspberry Pi web interface

The Raspberry Pi Zero 2 W web application also contains a DFU workflow using
Newtmgr. This interface is a legacy/experimental component and may require
adaptation to the current binary BLE protocol. Prefer the desktop application
or `dfu_ble_dongle.py` for the current firmware.

## Troubleshooting

### `west: command not found`

Open a terminal from the nRF Connect v2.8.0 toolchain environment or activate
the SDK environment before building.

### Application or board not found

Confirm that west uses the correct workspace:

```bash
west topdir
west config manifest.path
```

The manifest path should point to `self-balancing-robot`, and the application
must be built from `Sources/applications/app`.

### Stale CMake, Kconfig, or devicetree configuration

Remove cached build configuration with a pristine rebuild:

```bash
west build -p always
```

### DFU package is missing

Confirm that the build completed successfully and that MCUboot is enabled.
Then check:

```text
Sources/applications/app/build/dfu_application.zip
```

### Memory report

After a successful build:

```bash
cd Sources/applications/app/build
ninja partition_manager_report
```
