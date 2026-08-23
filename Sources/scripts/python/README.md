# Raspberry Pi Zero 2 W Camera AP Provisioner

**provision_rpi_os_sd.py** python script automatically provisions an SD card for a Raspberry Pi Zero 2 W to run as a standalone wireless access point with a live camera RTSP/WebRTC stream.

---

## How It Works

The script automates the entire setup process by performing the following actions:
* Downloads the latest **Raspberry Pi OS Lite 32-bit** image.
* Downloads **MediaMTX** (ARMv7) from GitHub to handle high-performance video streaming.
* Flashes the image directly to the target SD card block device.
* Configures the filesystems on the SD card to set up:
  * A WPA2 Wi-Fi access point (`wlan0`) with a static IP address (`192.168.4.1`).
  * Headless **SSH access** enabled by default.
  * Automatic hardware-accelerated camera detection and an RTSP stream via MediaMTX (`rtsp://192.168.4.1:8554/camera`).

---

## Requirements & Fetched Tools

### Host Requirements
* A running **Linux** host system.
* **Root privileges** (`sudo`).
* Mandatory host command-line tools: `lsblk`, `mount`, `umount`, and `openssl`.

### Downloaded Assets
* **Raspberry Pi OS Lite 32-bit (armhf)** and its corresponding `.sha256` checksum file from official Raspberry Pi repositories.
* **MediaMTX** (`_linux_armv7.tar.gz`) from the official BlueEnviron GitHub releases API.

---

## Usage

1. Connect your micro-SD card reader with the SD card inserted into your Linux host.
2. Identify your target block device name using `lsblk` (e.g., `/dev/sdb`, **do not** choose a partition like `/dev/sdb1`).
3. Run the script with `sudo`, providing the target device:

```bash
sudo python3 provision_rpi_camera_sd.py --device /dev/sdX

