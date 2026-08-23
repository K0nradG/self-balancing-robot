# Environment setup

[Project overview](../README.md) |
**Environment setup** |
[Building and flashing](building_and_running_fw.md)

This project is a Zephyr module and a west manifest repository based on
**nRF Connect SDK v2.8.0** and **nRF Connect toolchain v2.8.0**.

The environment can be configured through the nRF Connect extension for
Visual Studio Code or entirely from a terminal. Both workflows produce the
same west workspace and use the same build commands.

## Prerequisites

Install:

- [nRF Connect SDK v2.8.0](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/2.8.0/nrf/installation/install_ncs.html)
- nRF Connect toolchain v2.8.0
- [nRF Connect for VS Code](https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect-extension-pack)
- Git

> [!NOTE]
> **nRF Connect for Desktop** and its Toolchain Manager can be used to install
> the SDK and open a terminal with the toolchain environment. The build
> integration inside the editor is provided by **nRF Connect for VS Code**.

## Option 1: Configure an existing NCS workspace in VS Code

Use this method if nRF Connect SDK v2.8.0 is already installed in a directory
such as `~/ncs/v2.8.0`.

1. Clone this repository into the SDK workspace:

   ```bash
   cd ~/ncs/v2.8.0
   git clone https://github.com/K0nradG/self-balancing-robot.git
   ```

2. Open `~/ncs/v2.8.0` in Visual Studio Code.
3. Open the **nRF Connect** extension.
4. Select **Manage West Workspace** and then
   **Switch West Manifest Repository**.
5. Select `self-balancing-robot/west.yml`.
6. Run **West Update** when prompted.
7. Select nRF Connect SDK and toolchain v2.8.0 for the workspace.

The resulting layout should contain:

```text
~/ncs/v2.8.0/
├── .west/
├── nrf/
├── zephyr/
└── self-balancing-robot/
```

## Option 2: Configure a new workspace from the terminal

Use a terminal in which the nRF Connect SDK v2.8.0 toolchain environment is
active. This can be a terminal opened from Toolchain Manager or the nRF
Connect extension.

```bash
west init \
  -m https://github.com/K0nradG/self-balancing-robot.git \
  --mr main \
  self-balancing-robot-workspace
cd self-balancing-robot-workspace
west update
west zephyr-export
```

The manifest imports the matching nRF Connect SDK revision automatically.
The repository is available at:

```text
self-balancing-robot-workspace/self-balancing-robot/
```

## Option 3: Switch an existing workspace from the terminal

If the repository has already been cloned into `~/ncs/v2.8.0`, the manifest
can be selected without using VS Code:

```bash
cd ~/ncs/v2.8.0
west config manifest.path self-balancing-robot
west update
west zephyr-export
```

Verify the environment before building:

```bash
west --version
west topdir
```

`west topdir` should print the root of the workspace containing
`self-balancing-robot`.

Continue with [Building and flashing](building_and_running_fw.md).
