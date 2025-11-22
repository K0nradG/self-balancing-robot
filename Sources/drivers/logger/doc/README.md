# Connecting to a Bluetooth Device Using `bluetoothctl`

## Introduction

This guide explains how to connect to a Bluetooth device using `bluetoothctl`, enable scanning, and access GATT attributes.

## Requirements

- Linux with `bluetoothctl` installed (part of the `bluez` package)
- Bluetooth enabled on the computer
- Bluetooth device MAC address (e.g., `C6:57:6E:61:F2:06`)

## Steps

### 1. Start `bluetoothctl`

Open a terminal and enter:

```sh
bluetoothctl
```

### 2. Turn on Bluetooth

Enter the command:

```sh
power on
```

### 3. Enable device scanning

To find the Bluetooth device, enable scanning:

```sh
scan on
```

Wait about 10 seconds for the system to detect the device.

### 4. Connect to the device

Once the device is detected, connect to it using its MAC address:

```sh
connect C6:57:6E:61:F2:06
```

### 5. Enter the GATT menu

After successfully connecting, enter the GATT menu:

```sh
menu gatt
```

### 6. List device attributes

To see the list of GATT attributes:

```sh
list-attributes
```

### 7. Select a specific attribute

If you want to access a particular attribute, select it using its path, in our case it is an attribute which declares RX characteristic of NUS service.

```sh
select-attribute /org/bluez/hci0/dev_C6_57_6E_61_F2_06/service0010/char0011
```

### **8. Enable notyfications**

To perform this, type:

```sh
notify on
```



## Sending Data to the Device (TX Characteristic)

You can use the TX characteristic to send data to the device, for example, to update the parameters of a PID controller.

### Data Format:

```
k[k_gain]i[i_gain]d[d_gain]s[setpoint]
```

Where:

- `k_gain` – proportional gain (Kp)
- `i_gain` – integral gain (Ki)
- `d_gain` – derivative gain (Kd)
- `setpoint` – target setpoint value

### Example:

```
k3.4i2.1d7.3s-90
```

The above values set:

- `Kp = 3.4`
- `Ki = 2.1`
- `Kd = 7.3`
- `Setpoint = -90`
