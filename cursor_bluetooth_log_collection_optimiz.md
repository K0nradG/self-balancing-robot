# Bluetooth log collection optimization
_Exported on 8/18/2026 at 13:48:11 GMT+2 from Cursor (3.2.11)_

---

**User**

hi I have a task for you

I have self balancing robot system, which works in the 2ms interval control loop
i want to create an app to collect logs from my system uusing bluetooth le
i tried using BLE NUS but it can't handle the speed of data sending 

what should i do ? how can I optimize it?
or maybe I should create custok service which will be faster ?

https://github.com/K0nradG/self-balancing-robot 
here is the repository of the project 

can you help me with this ?

---

**Cursor**

Yes. The main bottleneck is not NUS itself—it is how the telemetry is encoded and sent.

NUS is already a custom GATT service. Replacing it with another notification-based service will not inherently improve BLE throughput.

From your repository:

- Ten floating-point values are formatted as text in the control path: [robot_controller.cpp](https://github.com/K0nradG/self-balancing-robot/blob/main/Sources/robot_control/src/robot_controller.cpp)
- `ble_send()` immediately calls `bt_nus_send()` without buffering or flow control: [ble_logger_service.cpp](https://github.com/K0nradG/self-balancing-robot/blob/main/Sources/drivers/bluetooth/src/ble_logger_service.cpp)
- You already request 2M PHY, Data Length Extension, and MTU exchange: [ble_logger_setup.cpp](https://github.com/K0nradG/self-balancing-robot/blob/main/Sources/drivers/bluetooth/src/ble_logger_setup.cpp)
- Preferred connection intervals are currently commented out: [board configuration](https://github.com/K0nradG/self-balancing-robot/blob/main/Sources/applications/app/boards/nrf52840dongle_nrf52840.conf)

Recommended design:

1. Keep the 2 ms control loop independent from BLE.
2. In each iteration, copy measurements into a fixed-size ring buffer.
3. Have a lower-priority telemetry thread drain that buffer.
4. Use packed binary samples—not `snprintf()` with `%f`.
5. Batch several samples into each BLE notification.
6. Add sequence numbers, timestamps, and a dropped-sample counter.

For example:

```cpp
struct __packed TelemetrySample {
    uint32_t timestamp_us;
    float balance_angle;
    float rotation_angle;
    float wheel_speed_0;
    float wheel_speed_1;
    float pwm_0;
    float pwm_1;
};
```

This sample is 28 bytes. Eight samples need 224 bytes, fitting into one 244-byte ATT notification when MTU is 247. You would send one notification every 16 ms while retaining all 500 Hz samples.

Your current text representation can easily require 100–180 bytes per sample:

- Text: approximately 50–90 kB/s at 500 Hz
- Binary: approximately 14–24 kB/s
- Quantized `int16_t`: potentially below 10–15 kB/s

Additional changes:

- Request a 7.5–15 ms connection interval with latency 0.
- Increase TX buffer counts, not only buffer sizes.
- Handle `-ENOMEM` from `bt_nus_send()` instead of dropping silently.
- Disable `CONFIG_LOG_MODE_IMMEDIATE` and high-rate UART text logging while collecting telemetry.
- Keep NUS for commands and optionally create a separate binary telemetry characteristic for cleaner protocol separation.
- Use notifications, not indications.
- On the PC/Raspberry Pi, enqueue notification data immediately and write to disk in batches; do not parse or print inside the BLE callback.

A custom telemetry service is worthwhile for structure and completion callbacks, but binary encoding, batching, connection parameters, and asynchronous buffering provide the actual speed improvement.

---

**User**

okay can you implement the changes here ?

---

**Cursor**

**Choose implementation location**

> **The open workspace is `ncs-door-lock-and-access-control`, not the robot repository. Where should I implement the telemetry changes?**
> - Clone the robot repository beside this workspace (Recommended)
> - Use an existing local robot checkout; I’ll provide its path
> - Add the robot telemetry implementation to the current workspace
>
> Answer: i want you to impelemtn this changes just here in chat as a text okay

Yes. Below is a concrete implementation that keeps NUS but changes telemetry to queued, batched binary data.

## 1. Add `telemetry.h`

```cpp
#pragma once

#include <cstdint>

struct TelemetrySample {
    uint32_t timestamp_us;

    float balance_setpoint;
    float balance_angle;
    float rotation_setpoint;
    float rotation_angle;

    float target_speed_0;
    float target_speed_1;
    float measured_speed_0;
    float measured_speed_1;

    float pwm_0;
    float pwm_1;
};

static_assert(sizeof(TelemetrySample) == 44);

void telemetry_submit(TelemetrySample const &sample);
```

## 2. Add `telemetry.cpp`

This thread sends five 2 ms samples in one notification every 10 ms.

```cpp
#include "telemetry.h"
#include "ble_service.h"

#include <cerrno>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

namespace {

constexpr uint32_t TELEMETRY_MAGIC = 0x314D4C54; // "TLM1"
constexpr uint16_t TELEMETRY_VERSION = 1;
constexpr size_t SAMPLES_PER_FRAME = 5;

struct TelemetryFrame {
    uint32_t magic;
    uint16_t version;
    uint16_t sample_count;
    uint32_t sequence;
    uint32_t dropped_samples;
    TelemetrySample samples[SAMPLES_PER_FRAME];
};

static_assert(sizeof(TelemetryFrame) == 236);

K_MSGQ_DEFINE(
    telemetry_queue,
    sizeof(TelemetrySample),
    128,
    alignof(TelemetrySample));

atomic_t dropped_samples;
uint32_t sequence;

void telemetry_thread(void *, void *, void *)
{
    TelemetryFrame frame{};
    bool frame_pending = false;

    while (true) {
        k_sleep(K_MSEC(10));

        if (!frame_pending) {
            frame.magic = TELEMETRY_MAGIC;
            frame.version = TELEMETRY_VERSION;
            frame.sample_count = 0;
            frame.sequence = sequence;
            frame.dropped_samples =
                static_cast<uint32_t>(atomic_get(&dropped_samples));

            while (frame.sample_count < SAMPLES_PER_FRAME) {
                auto &sample = frame.samples[frame.sample_count];

                if (k_msgq_get(&telemetry_queue, &sample, K_NO_WAIT) != 0) {
                    break;
                }

                ++frame.sample_count;
            }

            frame_pending = frame.sample_count != 0;
        }

        if (!frame_pending) {
            continue;
        }

        const size_t frame_size =
            offsetof(TelemetryFrame, samples) +
            frame.sample_count * sizeof(TelemetrySample);

        const int error = ble_send_binary(
            reinterpret_cast<uint8_t const *>(&frame),
            static_cast<uint16_t>(frame_size));

        if (error == 0) {
            ++sequence;
            frame_pending = false;
        } else if (error != -ENOMEM) {
            atomic_add(&dropped_samples, frame.sample_count);
            frame_pending = false;
        }
    }
}

K_THREAD_DEFINE(
    telemetry_thread_id,
    2048,
    telemetry_thread,
    nullptr,
    nullptr,
    nullptr,
    5,
    0,
    0);

} // namespace

void telemetry_submit(TelemetrySample const &sample)
{
    if (k_msgq_put(&telemetry_queue, &sample, K_NO_WAIT) != 0) {
        atomic_inc(&dropped_samples);
    }
}
```

## 3. Extend `ble_service.h`

```cpp
int ble_send_binary(uint8_t const *data, uint16_t length);
```

## 4. Add binary sending to `ble_logger_service.cpp`

```cpp
#include <errno.h>

int ble_send_binary(uint8_t const *data, uint16_t length)
{
    if (!get_con_status() || !get_notif_status()) {
        return -ENOTCONN;
    }

    if (data == nullptr || length == 0) {
        return -EINVAL;
    }

    // 244-byte payload with ATT MTU 247.
    if (length > 244) {
        return -EMSGSIZE;
    }

    return bt_nus_send(nullptr, data, length);
}
```

Keep the existing text `ble_send()` for configuration messages and commands.

## 5. Submit samples from `robot_controller.cpp`

Add:

```cpp
#include "telemetry.h"
```

After calculating the controller outputs:

```cpp
TelemetrySample const sample{
    .timestamp_us = k_uptime_get_32() * 1000U,

    .balance_setpoint =
        m_balance_setpoint * radian_degrees / pi,
    .balance_angle =
        imu_data.angle_balance * radian_degrees / pi,
    .rotation_setpoint =
        m_rotate_setpoint_ramp.get_current_value() * radian_degrees / pi,
    .rotation_angle =
        rotation_angle * radian_degrees / pi,

    .target_speed_0 = target_speed0,
    .target_speed_1 = target_speed1,
    .measured_speed_0 =
        encoders_data.encoder_0.angular_velocity_rad_s,
    .measured_speed_1 =
        encoders_data.encoder_1.angular_velocity_rad_s,

    .pwm_0 = m_pwm0,
    .pwm_1 = m_pwm1,
};

telemetry_submit(sample);
```

Remove the existing formatted high-rate log:

```cpp
robot_control_logger.platform_log(
    LOG_LEVEL::INF,
    "bs: %f, ab: %f, ...");
```

## 6. BLE configuration

Use an ATT MTU of 247 and add more TX buffers:

```conf
CONFIG_BT_DATA_LEN_UPDATE=y
CONFIG_BT_CTLR_DATA_LENGTH_MAX=251
CONFIG_BT_BUF_ACL_TX_SIZE=251
CONFIG_BT_BUF_ACL_TX_COUNT=12

CONFIG_BT_L2CAP_TX_MTU=247
CONFIG_BT_L2CAP_TX_BUF_COUNT=12
CONFIG_BT_ATT_TX_COUNT=12

CONFIG_BT_PHY_UPDATE=y

CONFIG_BT_PERIPHERAL_PREF_MIN_INT=6
CONFIG_BT_PERIPHERAL_PREF_MAX_INT=12
CONFIG_BT_PERIPHERAL_PREF_LATENCY=0
CONFIG_BT_PERIPHERAL_PREF_TIMEOUT=400
CONFIG_BT_GAP_AUTO_UPDATE_CONN_PARAMS=y

CONFIG_LOG_MODE_IMMEDIATE=n
```

Some buffer-count names vary between NCS versions, so confirm them with `menuconfig`.

## 7. Python collector using Bleak

```python
import asyncio
from bleak import BleakClient

ADDRESS = "XX:XX:XX:XX:XX:XX"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

queue = asyncio.Queue(maxsize=1000)
callback_drops = 0


def notification_handler(_, data: bytearray):
    global callback_drops

    try:
        queue.put_nowait(bytes(data))
    except asyncio.QueueFull:
        callback_drops += 1


async def file_writer():
    with open("robot_telemetry.bin", "wb", buffering=1024 * 1024) as output:
        while True:
            frame = await queue.get()
            output.write(frame)
            queue.task_done()


async def main():
    writer = asyncio.create_task(file_writer())

    async with BleakClient(ADDRESS) as client:
        await client.start_notify(NUS_TX_UUID, notification_handler)
        print("Collecting telemetry. Press Ctrl+C to stop.")

        try:
            while True:
                await asyncio.sleep(1)
                print(
                    f"queued={queue.qsize()}, "
                    f"callback_drops={callback_drops}"
                )
        finally:
            await client.stop_notify(NUS_TX_UUID)
            await queue.join()
            writer.cancel()


asyncio.run(main())
```

This reduces the transmission rate from 500 text notifications/s to about 100 binary notifications/s while preserving every 2 ms sample.

---

**User**

okay why we need this ?

constexpr uint32_t TELEMETRY_MAGIC = 0x314D4C54; // "TLM1"
constexpr uint16_t TELEMETRY_VERSION = 1;
what is thtat ?

---

**Cursor**

They are metadata that help the receiver recognize and decode a telemetry frame.

```cpp
constexpr uint32_t TELEMETRY_MAGIC = 0x314D4C54; // "TLM1"
```

This is a fixed signature at the beginning of every frame. On a little-endian nRF processor, those bytes appear as:

```text
54 4C 4D 31 → T L M 1
```

The receiver checks it to:

- Confirm it found the beginning of a frame.
- Reject corrupted or unexpected data.
- Recover synchronization if bytes are missing.

```cpp
constexpr uint16_t TELEMETRY_VERSION = 1;
```

This identifies the binary format. If you later add or reorder fields, increment it to `2`. The receiver can then select the correct decoder or reject an unsupported format.

Neither is strictly required because BLE preserves notification boundaries. However, they make saved binary logs safer and easier to diagnose.

For an extremely minimal protocol, remove both and keep only:

```cpp
struct TelemetryFrame {
    uint16_t sample_count;
    uint32_t sequence;
    uint32_t dropped_samples;
    TelemetrySample samples[5];
};
```

I recommend retaining them—the cost is only 6 bytes per notification.

---

**User**

i think TELEMETRY_MAGIC can stay but TELEMETRY_VERSION is not needed

---

**Cursor**

Agreed. `"TLM1"` already implicitly identifies format version 1. For a future incompatible format, change it to `"TLM2"`.

Use:

```cpp
constexpr uint32_t TELEMETRY_MAGIC = 0x314D4C54; // "TLM1"

struct TelemetryFrame {
    uint32_t magic;
    uint32_t sequence;
    uint32_t dropped_samples;
    uint16_t sample_count;
    uint16_t reserved;
    TelemetrySample samples[SAMPLES_PER_FRAME];
};
```

Keep `reserved` as zero for alignment and possible future use. The frame remains 236 bytes.

---

**User**

okay, i have got one more question, does the our changes in implementation handilng errors on the nus level ?

for example we are putting some dumy data so we know that this package was not complete or smth ?


---

**Cursor**

No dummy data is inserted. Instead, errors are detected using metadata:

- `magic`: verifies a valid frame.
- `sequence`: detects missing notification frames.
- `dropped_samples`: reports samples discarded because the firmware queue was full.
- `sample_count`: tells the receiver the expected frame size.

BLE also protects transmission with CRC and link-layer retransmissions. A partially received ATT notification is not delivered to the application—it receives the complete notification or nothing.

Current limitations:

- `bt_nus_send() == 0` means queued by the BLE stack, not confirmed by the receiver.
- NUS notifications have no application-level acknowledgement.
- `-ENOMEM` is retried; other send errors cause the samples to be dropped and counted.

The receiver should check:

```python
if magic != TELEMETRY_MAGIC:
    print("Invalid frame")

if sequence != expected_sequence:
    print(f"Missing frames: expected {expected_sequence}, got {sequence}")

expected_sequence = sequence + 1
```

For exact per-sample loss detection, add a sequence to every sample:

```cpp
struct TelemetrySample {
    uint32_t sample_sequence;
    uint32_t timestamp_us;
    // measurements...
};
```

That is better than inserting dummy samples: a sequence gap directly identifies missing data. If guaranteed end-to-end delivery is required, you need receiver acknowledgements and retransmission buffering; this reduces throughput.

---

**User**

okay, you are right

can you tell me why the current imp,ementaion (on the reposityroiy) can't send all data ?

it is sending logs with interval euqual to 1s 

---

**Cursor**

The current implementation is not collecting all 2 ms samples and sending them once per second. It only takes one snapshot every second.

In [`robot_controller.cpp`](https://github.com/K0nradG/self-balancing-robot/blob/main/Sources/robot_control/src/robot_controller.cpp), the logic is effectively:

```cpp
log_timer_ms += imu_data.time_dt * 1000;

if (log_timer_ms >= 1000) {
    log_timer_ms = 0;
    send_current_values();
}
```

With a 2 ms control loop, there are 500 samples per second. The implementation sends only the latest one:

```text
Samples 0–498: never stored
Sample 499: formatted and sent
```

Therefore, approximately 499 out of every 500 samples are never logged. This is sampling at 1 Hz, not transferring 500 Hz data in one-second batches.

Other limitations are:

- No queue or ring buffer.
- ASCII float formatting is relatively large and CPU-intensive.
- The formatted message is silently truncated at 243 characters by `LOG_MSG_MAX_SIZE`.
- `bt_nus_send()` errors are only printed; data is not queued or retried.
- The real NUS limit is negotiated ATT MTU minus 3, not `BLE_NUS_MAX_DATA_LEN`.
- A one-second interval easily fits BLE bandwidth, but it cannot contain the preceding 500 samples because they were never stored.

That is why the proposed implementation calls `telemetry_submit()` every 2 ms, stores each sample, and sends five binary samples together every 10 ms. Sampling and transmission are then separate operations.

---

**User**

okay, there is another question

i have got a python app which collects the data from robot
plotting them to visualise it etc..

i also have rpi zero 2w which has camera connected in it recogizethe the path and sends it to the robot's trajectory menager but now i have got robot connected to the python app

and how can i send trajecotyr data from the rpi to the robot ? what do you recommend ?

---

**Cursor**

The robot currently effectively assumes one BLE central. Therefore, the PC app and Raspberry Pi should not independently connect to it.

I recommend making the Raspberry Pi the communication gateway:

```text
Camera/path planner
        │
        ▼
Raspberry Pi ── BLE ── Robot
        │
     Wi-Fi
        │
        ▼
PC visualization app
```

The same BLE connection works in both directions:

- Robot → RPi: binary telemetry through NUS TX notifications.
- RPi → Robot: trajectory commands through NUS RX writes.
- RPi → PC: forward telemetry over WebSocket or TCP.
- PC → RPi: optional configuration and manual commands.

For trajectory commands, reliability is more important than maximum speed. Use:

- Write-with-response.
- Trajectory ID and point sequence number.
- Robot acknowledgement.
- A short buffered trajectory horizon.
- Timeout that safely stops the robot if updates stop.

Example trajectory packet:

```cpp
struct TrajectoryPoint {
    uint32_t trajectory_id;
    uint16_t point_index;
    uint16_t point_count;
    float distance_m;
    float rotation_rad;
    float target_speed_m_s;
};
```

RPi Python code can send it through NUS RX:

```python
import struct

NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

packet = struct.pack(
    "<IHHfff",
    trajectory_id,
    point_index,
    point_count,
    distance_m,
    rotation_rad,
    target_speed,
)

await robot.write_gatt_char(
    NUS_RX_UUID,
    packet,
    response=True,
)
```

Alternative: keep the PC as the BLE central and have the RPi send trajectory data to the PC over Wi-Fi; the PC then forwards it over BLE. This requires less restructuring, but trajectory control stops if the PC application disconnects.

For an autonomous robot, the RPi-as-gateway architecture is safer and cleaner. The PC should be an optional monitoring client, not part of the essential control path.

---

**User**

okay bot how we can handle 2 parrarel ble connection son the robot side ?

would't be better to communicate to rpi communicate with python app, 
app store it ahnd then send to the robot ?

---

**Cursor**

Yes—if the Python application is always running, your proposed architecture is simpler than supporting two BLE connections:

```text
RPi camera ── Wi-Fi/TCP ── Python app ── BLE ── Robot
                                │
                         store and visualize
```

The Python app would:

1. Maintain the existing BLE connection.
2. Receive and store robot telemetry.
3. Receive trajectory points from the RPi over TCP/WebSocket.
4. Forward them to the robot through NUS RX.

Use TCP or WebSocket between the RPi and Python app because trajectories require ordered, reliable delivery.

Example forwarding structure:

```python
trajectory_queue = asyncio.Queue()


async def robot_writer(robot):
    while True:
        trajectory = await trajectory_queue.get()

        await robot.write_gatt_char(
            NUS_RX_UUID,
            trajectory,
            response=True,
        )

        trajectory_queue.task_done()


async def handle_rpi(reader, writer):
    while data := await reader.readline():
        packet = encode_trajectory(data)
        await trajectory_queue.put(packet)
```

A dedicated queue ensures trajectory writes are serialized and do not block telemetry callbacks.

## Why not two BLE connections?

It is technically possible using:

```conf
CONFIG_BT_MAX_CONN=2
```

However, your current robot implementation assumes one connection:

- One global `my_conn`.
- One global notification-enabled flag.
- No distinction between PC and RPi commands.
- `bt_nus_send(nullptr, ...)` may send to every subscribed connection.
- Both connections share radio bandwidth.
- Two devices could issue conflicting commands.

Supporting it properly requires per-connection state, connection roles, command ownership, per-connection MTU handling, and command arbitration.

Therefore:

- For development with the PC always present: **RPi → Python → BLE robot** is the best short-term design.
- For fully autonomous operation: **RPi → BLE robot**, with the PC connecting to the RPi over Wi-Fi, is safer because the robot does not depend on the PC.

---

**User**

@/home/kogr/ncs/v3.3.0/self-balancing-robot 

on this branch there is an oython app to interact with robot over ble le

@/home/kogr/.cursor/projects/home-kogr-ncs-v3-3-0-ncs-door-lock-and-access-control/terminals/1.txt:49-68 

can you now implement the improvements which we are talikng about ?


---

**Cursor**

The firmware now captures every control-loop sample, batches five samples per BLE notification, and the Python app decodes all samples while throttling only the plot refresh. Text NUS commands and status messages remain compatible.

---

**User**

don'tbuild it now the can be toolchain errors

---

**Cursor**

Understood—I’ll stop at source-level verification and won’t run the firmware build. The available shell lacks the configured NCS toolchain anyway.

Implemented on local branch `work/introduce_robot_control_application`.

Changes include:

- 2 ms samples queued without blocking control loop.
- Five binary samples per 236-byte NUS notification.
- Sequence and dropped-sample tracking.
- Retry on BLE buffer exhaustion.
- Python binary decoding and CSV batch writes.
- Plot updates throttled to 50 Hz while recording every sample.
- Legacy text messages remain supported.
- Increased BLE buffers, enabled 2M PHY and faster intervals.
- Added three passing parser tests.

Firmware build was skipped as requested. Python syntax, tests, whitespace, and IDE lint checks pass. No commit was created.

The RPi-to-Python trajectory bridge is not included yet; this change covers the telemetry improvements.

---

**User**

okay now i have appropirate setup, i moved repository to the v2.8.0 repository and i am using 2.8 toolchain

@/home/kogr/ncs/v2.8.0/self-balancing-robot 

can you check wheter everytifk building coreclyt ?

---

**Cursor**

Build is successful with NCS/toolchain v2.8.0.

Verified:

- Firmware and MCUboot linked successfully.
- `telemetry.cpp` was compiled and linked.
- `merged.hex` generated.
- FLASH: 42.16%, RAM: 48.31%.
- Python telemetry tests: 3/3 passed.
- No linter or whitespace errors.

There are non-fatal configuration warnings. Most importantly, NCS 2.8 expects:

```conf
CONFIG_BT_BUF_ACL_TX_SIZE=251
```

instead of `502`. A 251-byte buffer still supports our 236-byte telemetry frame and removes both MCUmgr buffer warnings.

Other warnings concern `BOARD_ROOT`, USB CDC being forced on, and the HCI TX stack setting being ignored. They do not prevent the build. Earlier errors in the terminal were from previous attempts; the latest build completed successfully.

---

**User**

@telemetry.cpp (1-111) 

i am only weonering about buffor collecting data 

because for now the architecture is triggered by the irq from the mpu 6050 module

it triggerd control  loop then it triggers motor control etc...

it works with 2ms intervak bu tometime there is ome jitter so i think we shoudl watch the current samples (when the 5 samples come)

and then send them 

not with fixed 10ms interval (beacse sometimes there can be 5 sampels, sometimes 6 sometimes 4...etc 

you understand yes?)

---

**Cursor**

Yes, you are correct.

The current fixed 10 ms wake-up does not lose the sixth sample—it stays in the queue—but it can create uneven batches and accumulating latency when the control-loop timing jitters.

A better design is event-driven:

1. Control loop submits samples non-blockingly.
2. Telemetry thread blocks until five samples have arrived.
3. Immediately send that five-sample frame.
4. No fixed 10 ms timer.

Conceptually:

```cpp
while(true)
{
    frame.magic           = telemetry_magic;
    frame.sequence        = frame_sequence;
    frame.dropped_samples = atomic_get(&dropped_samples);
    frame.sample_count    = samples_per_frame;

    for(size_t i = 0; i < samples_per_frame; ++i)
    {
        k_msgq_get(&telemetry_queue, &frame.samples[i], K_FOREVER);
    }

    int err;
    do
    {
        err = ble_send_binary(
            reinterpret_cast<uint8_t const*>(&frame),
            sizeof(frame));

        if(err == -ENOMEM)
        {
            k_sleep(K_MSEC(1));
        }
    } while(err == -ENOMEM);

    if(err == 0)
    {
        frame_sequence++;
    }
    else if(err != -ENOTCONN)
    {
        atomic_add(&dropped_samples, samples_per_frame);
    }
}
```

Therefore, if samples arrive with jitter at:

```text
0.0, 2.1, 3.9, 6.3, 8.2 ms
```

the frame is sent at approximately 8.2 ms, immediately after the fifth sample. The timestamps still preserve the real control-loop timing.

This is preferable for your IRQ-driven architecture. The telemetry thread must remain lower priority than the control loop.

---

**User**

okay, can you implement this event driven apporach ?

---

**Cursor**

The event-driven source compiled and linked successfully. The verification then hit a stale generated-build reference to the old toolchain during image signing, so I’m regenerating the build directory with only the v2.8 toolchain.

Implemented the event-driven approach.

- Telemetry thread blocks on `k_msgq_get(..., K_FOREVER)`.
- Sends immediately after exactly five samples arrive.
- No fixed 10 ms timer.
- `-ENOMEM` retries use a 1 ms backoff.
- Control-loop submission remains non-blocking.
- Real sample timing remains available through timestamps.

A pristine NCS v2.8 build completed successfully and generated `merged.hex`. No linter or whitespace errors.

