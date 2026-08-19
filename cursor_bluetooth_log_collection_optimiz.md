# Bluetooth log collection optimization
_Exported on 8/18/2026 at 14:43:40 GMT+2 from Cursor (3.2.11)_

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

---

**User**

okay i have one more question did you modfify the python app so it could "read" and plot data send by robot over ble ?

---

**Cursor**

Yes.

The Python app now:

- Detects binary telemetry using the `"TLM1"` magic.
- Decodes the frame header and all five samples.
- Validates frame size and sample count.
- Detects missing frames using `sequence`.
- Reads the firmware dropped-sample counter.
- Records every decoded sample to CSV.
- Updates plots at 50 Hz using the newest sample, preventing GUI overload.
- Continues supporting legacy text messages for battery, PID parameters, and commands.

The changes are in:

- `robot_control_app/ble_worker/ble_worker.py`
- `robot_control_app/ble_worker/data_processor.py`
- `robot_control_app/ble_worker/test_data_processor.py`

The binary decoder tests pass.

---

**User**

Records every decoded sample to CSV.

every sample ? you mean every 5 samples ?

i mean when the saving samples to the .csv file starts and ends ?

what do you mean by saying legacy texts ?

Continues supporting legacy text messages for battery, PID parameters, and commands.

maybe we should remove it and jus use binary data exchange everywhere across the app ? (then we shoudl modify the platform log , yes ?)

---

**Cursor**

“Every sample” means every control-loop sample.

One BLE frame contains five samples, and the CSV writer creates five separate rows:

```text
BLE frame 10
 ├─ sample 50 → CSV row
 ├─ sample 51 → CSV row
 ├─ sample 52 → CSV row
 ├─ sample 53 → CSV row
 └─ sample 54 → CSV row
```

The plot does not update for every sample. It uses the newest sample at up to 50 Hz to avoid overloading PyQt, while CSV stores all 500 Hz samples.

Current recording lifecycle:

- Starts on the first telemetry frame after “Auto Record” is enabled.
- Stops writing when disabled, but currently leaves the file open.
- Resumes in the same file if re-enabled.
- Closes on BLE disconnect or application shutdown.

It would be cleaner to close the file when recording is disabled and create a new file when enabled again.

“Legacy text” means only high-rate telemetry was converted to binary. The same NUS connection still carries text for:

- PID parameter responses.
- Battery messages.
- Trajectory completion acknowledgement.
- State/configuration commands.
- Diagnostic logs.

Using binary for all structured communication is a good long-term design, but `platform_log()` should not become the binary protocol. It handles arbitrary human-readable diagnostic messages.

Instead, separate them:

- `platform_log()` → UART/debug logs only.
- `ble_send_message(type, payload)` → structured binary application protocol.

For example:

```cpp
enum class Message_Type : uint8_t
{
    TELEMETRY,
    BATTERY_STATUS,
    PID_PARAMETERS,
    TRAJECTORY_COMMAND,
    TRAJECTORY_COMPLETE,
    STATE_COMMAND,
    ERROR_STATUS,
};
```

Both directions could use a common binary header containing magic, message type, payload length, and sequence. This requires updating the firmware command parser and Python command encoder, not only `platform_log()`. Low-rate text is not causing the throughput problem, so this should be treated as a protocol refactor rather than a necessary telemetry optimization.

---

**User**

Using binary for all structured communication is a good long-term design, but platform_log() should not become the binary protocol. It handles arbitrary human-readable diagnostic messages.

okay but it will be "decryped" from binary to human readable text so  we can communicate by using the binary, yes ?

---

**Cursor**

Yes. More precisely, it is “decoded,” not decrypted—binary encoding provides no security.

Example firmware payload:

```cpp
struct Battery_Status
{
    uint16_t millivolts;
};
```

On the wire, `3300 mV` becomes two binary bytes. Python decodes it:

```python
millivolts = struct.unpack("<H", payload)[0]
text = f"Battery: {millivolts} mV"
```

The application can then display human-readable text, plots, and values.

For diagnostic logs, there are two options:

1. Keep UTF-8 text inside a binary message envelope. Simple, but no bandwidth reduction.
2. Send structured event IDs and arguments, then let Python generate the text. Efficient, but firmware and Python must share an event schema.

Recommended:

- Numeric telemetry, battery, PID, commands and acknowledgements → structured binary.
- Debug/error logs → text payload or UART.
- Python decodes structured binary and generates readable UI text.

So yes, communication can be fully binary while the user still sees human-readable information.

---

**User**

okay so can you implement this approach ?

---

**Cursor**

**Binary protocol scope**

> **Should the migration intentionally break compatibility with existing text commands, or support both protocols temporarily?**
> - Binary-only protocol across firmware and Python (Recommended) ✔
> - Temporarily accept both binary and legacy text commands

> **How should arbitrary diagnostic logs from `platform_log()` reach the Python application?**
> - Binary LOG message containing UTF-8 text (Recommended) ✔
> - Keep diagnostics on UART only; BLE carries structured messages only
> - Convert diagnostics to binary event IDs and typed arguments

**Client migration scope**

> **The repository also contains the Raspberry Pi NUS client/web server. Should the binary-only migration update it in the same change?**
> - Update firmware, desktop GUI, and Raspberry Pi client (Recommended)
> - Update firmware and desktop GUI only; leave Raspberry Pi client incompatible temporarily
>
> Answer: no, for now don't look at this, this web server will be removed in the future, becasue we have the python app now

I’ve narrowed the migration to the firmware and desktop Python app only. It will be binary-only, while diagnostic logs will use a typed binary envelope containing UTF-8 text; the deprecated Raspberry Pi web server will remain untouched.


---

**User**

<timestamp>Tuesday, Aug 18, 2026, 2:07 PM (UTC+2)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: subagent
status: success
task_id: 7fc32775-9ce5-4e0e-bbc6-c4e27cba8970
title: Review protocol architecture
detail: <user_visible_high_level_summary>
Reviewed the v2.8.0 self-balancing-robot repo’s hybrid NUS design (binary telemetry TX + text commands/status). One NUS characteristic per direction can work with magic-based dispatch, but full binary RX requires replacing the null-terminated text parser. Key pitfalls: missing MTU clamp in `ble_send_binary`, struct casting without explicit LE encoding, shared TX buffer contention with `Logger`, and DFU/SMP radio buffer overlap during firmware updates.
</user_visible_high_level_summary>
<response>
## Current State

The repo is **partially converted**, not fully binary:

| Direction | Implemented today | Still text |
|-----------|-------------------|------------|
| **NUS TX** (robot → host) | Binary telemetry frames (`TLM1`, 236 B) in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/robot_control/src/telemetry.cpp` | Status/logs via `Logger` → `ble_send()` in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/logger/include/logger.h`; PID dumps, battery, trajectory ACK in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/robot_control/src/robot_controller.cpp`, `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/dfu_smp/src/dfu_ble.cpp` |
| **NUS RX** (host → robot) | Entirely ASCII prefix routing in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/bluetooth/src/ble_logger_service.cpp` → `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/robot_control/src/robot_controller.cpp` | — |

The Python app mirrors this hybrid model in `/home/kogr/ncs/v2.8.0/self-balancing-robot/robot_control_app/ble_worker/ble_worker.py`: `startswith(b"TLM1")` → binary decoder, else UTF-8 text parser.

The design discussion in `/home/kogr/ncs/v2.8.0/self-balancing-robot/cursor_bluetooth_log_collection_optimiz.md` proposes completing the migration while keeping NUS (not a new GATT service) and optionally moving trajectory commands to binary writes with `response=True`.

---

## Recommended Minimal Architecture

Use **one envelope on each NUS characteristic**, with message-type dispatch. Do not rely on casting C structs to wire bytes.

### Wire envelope (both directions, 8 bytes)

```
[magic: u32 LE] "TLM1" TX telemetry, "RBTX" TX status, "RBRX" RX commands
[msg_type: u8]
[payload_len: u16 LE]
[reserved: u8 = 0]
[payload: payload_len bytes]
```

Keep message types small and explicit:

**TX (notifications, no app-level ACK)**
- `0x01` — telemetry batch (current 5× sample payload)
- `0x02` — PID parameters blob
- `0x03` — battery/status
- `0x04` — trajectory completed ACK
- `0x05` — error/status code (optional)

**RX (write-with-response)**
- `0x10` — state machine start/stop
- `0x11` — regulator/PID tuning
- `0x12` — trajectory point
- `0x13` — get regulator params
- `0x20` — DFU start/skip (replaces `"db"` / `"ds"`)

**Serialization rules**
- Firmware: `sys_put_le16/32/64`, `sys_put_float` (or memcpy of IEEE754 after explicit LE write); never `reinterpret_cast<struct*>(&frame)` for wire format.
- Python: `struct.Struct("<IBHB")` header + typed payloads; same layouts in a shared schema file.
- Mark wire structs `__packed` only where needed; prefer natural alignment inside payload and explicit encoding for the header.

**Threading (keep what works)**
- Control loop: non-blocking `telemetry_submit()` (already in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/robot_control/src/robot_controller.cpp`).
- Telemetry thread: block until N samples, retry only on `-ENOMEM` (already in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/robot_control/src/telemetry.cpp`).
- Host: BLE callback enqueues raw bytes; decode/write CSV off the hot path (already mostly done in `/home/kogr/ncs/v2.8.0/self-balancing-robot/robot_control_app/ble_worker/data_processor.py`).

**DFU boundary**
- Keep **MCUmgr SMP on its own GATT service** (`/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/dfu_smp/src/dfu_ble.cpp`, `/home/kogr/ncs/v2.8.0/self-balancing-robot/dfu_utils/dfu_operations.py`). Do not tunnel firmware images over NUS.
- NUS only carries **DFU mode selection** (start/skip), then host switches to SMP.
- On DFU start: stop telemetry thread / set a “DFU active” gate before entering the DFU wait loop.

---

## Topic-by-Topic Review

### 1. Zephyr / NUS MTU constraints

- Effective ATT payload = **negotiated MTU − 3**. `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/bluetooth/src/ble_logger_setup.cpp` logs this correctly on exchange.
- Current telemetry frame is **236 B**, which fits the common **MTU 247 → 244 B payload** budget even on conservative centrals.
- Board config in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/applications/app/boards/nrf52840dongle_nrf52840.conf` sets `CONFIG_BT_L2CAP_TX_MTU=498` and `CONFIG_BT_BUF_ACL_TX_SIZE=502`. That exceeds what many centrals negotiate and triggered MCUmgr buffer warnings in your own build notes; **251-byte ACL buffers / MTU 247** are sufficient for 236-byte frames.
- **Gap:** `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/bluetooth/src/ble_logger_service.cpp` `ble_send_binary()` has **no MTU or 244-byte clamp** (the chat proposal had one). A future larger message (PID dump binary) could fail silently or return `-ENOMEM` depending on stack state.
- `Logger` caps at **244 chars** (`LOG_MSG_MAX_SIZE` in `logger.h`) — aligned with ATT, but text PID strings in `send_PID_controllers_parameters()` can be ~200+ bytes and contend with telemetry on the same TX path.

### 2. Endian / alignment safety

- `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/robot_control/include/telemetry.h` and `telemetry.cpp` use native struct layout + `reinterpret_cast` send. Works today on nRF52840 (little-endian, 4-byte aligned) and matches Python `"<I10f"` / `"<IIIHH"`.
- **Pitfall:** not portable, and `-Wpadded`/different toolchains could break `static_assert(sizeof == 236)`.
- Good precedent in-repo: `struct __packed IdentificationFrame` in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/model_identification/src/model_identification.cpp` — but that path uses UART, not NUS.
- **Recommendation:** explicit LE encode/decode for the header at minimum; keep floats as LE IEEE754 with `memcpy` after validation.

### 3. Packet header design

Current header (already deployed):

```
magic u32 | sequence u32 | dropped u32 | sample_count u16 | reserved u16 | samples[5]
```

Strengths:
- Magic gives stream resync in logged files.
- `sequence` + `dropped_samples` expose firmware-side loss.
- Python validates magic, reserved, sample_count, and total length (`/home/kogr/ncs/v2.8.0/self-balancing-robot/robot_control_app/ble_worker/data_processor.py`).

Weaknesses for full binary:
- Header is telemetry-specific; status/PID/trajectory-ACK still bypass it as raw text.
- Firmware always sends **fixed 236 B** with `sample_count = 5`, while Python accepts `1..5` — harmless but inconsistent.
- No **per-sample sequence** — frame gaps detect notification loss, not individual 2 ms sample loss inside a reconnect.
- `timestamp_us = k_uptime_get_32() * 1000` in `robot_controller.cpp` wraps and is ms-quantized, not true µs.

For full binary, extend with a **1-byte `msg_type`** after magic (or use separate magics per message class) rather than overloading `TLM1` for everything.

### 4. Write-with-response semantics

- Host already uses `response=True` in `/home/kogr/ncs/v2.8.0/self-balancing-robot/robot_control_app/ble_worker/ble_worker.py` and appends `\n` to text commands.
- That only guarantees **ATT write success**, not “command applied.”
- Firmware handles writes synchronously in `nus_data_received()` with **no application ACK** and no ATT error mapping for bad payloads.
- Text parser silently ignores unknown keys (`default: break` in `parse_nus_data()`); binary should return **`msg_type=0x05` error NOTIFY** or set a known error bit — otherwise the host cannot distinguish “accepted” from “dropped.”
- For trajectory (planned in chat): add **`trajectory_id` + `point_index` + optional NOTIFY ack**; write-with-response alone is insufficient for “robot reached point N.”

### 5. DFU coexistence

- Boot flow (`/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/dfu_smp/src/dfu_ble.cpp`): SMP advertising starts immediately; robot waits on NUS text `"db"`/`"ds"`.
- DFU transfer uses **separate SMP GATT service** via `smpmgr` (`/home/kogr/ncs/v2.8.0/self-balancing-robot/robot_control_app/dfu_worker/dfu_worker.py`), not NUS.
- **Shared resource contention:** both paths use the same ACL TX buffer pool (`CONFIG_BT_BUF_ACL_TX_COUNT` differs between logger board conf and DFU defconfig in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/dfu_smp/Kconfig.dfu_smp.defconfig`). High-rate telemetry during SMP upload can cause `-ENOMEM` on either path.
- **Recommendation:** gate telemetry off on DFU start; keep DFU bootstrap as a **small binary or ASCII command** outside the high-rate path; do not interleave SMP and NUS bulk traffic.

### 6. Protocol error handling

| Layer | Today | Needed for full binary |
|-------|-------|------------------------|
| ATT | Full notification or nothing; write response on RX | Same |
| Firmware TX | Retry `-ENOMEM`; count drops; increment sequence on success | Add MTU check; optionally expose `bt_nus_send` fail count |
| Firmware RX | Null-terminate, log as string, silent ignore | Length-checked binary parser; reject `payload_len > len-8`; no `char*` assumptions |
| Host TX | Exception log on write fail | Serialize commands through one asyncio queue (chat already suggested for trajectory) |
| Host RX | `ValueError` on bad telemetry; text fallback | Dispatch on magic+msg_type; never UTF-8-decode binary |

Critical RX blocker in `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/bluetooth/src/ble_logger_service.cpp`:

```81:96:/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/bluetooth/src/ble_logger_service.cpp
static void
nus_data_parser(const uint8_t* data, uint16_t len)
{
 ...
 static char received_data[BLE_NUS_MAX_DATA_LEN + 1];
 memset(received_data, 0, sizeof(received_data));
 memcpy(received_data, data, len);
 received_data[len] = '\0';

 data_selector(received_data);
}
```

Any binary RX payload containing **`0x00` truncates**; any binary trajectory/float field can break parsing. Full binary RX **requires replacing this path**, not extending it.

### 7. Can one NUS characteristic carry all message types?

**Yes, with discipline — one characteristic per direction.**

**TX (notify):** Already viable. Magic dispatch (`TLM1` vs text) in `ble_worker.py` works because BLE preserves notification boundaries. Rules:
- Every binary message starts with a **unique magic** (avoid ASCII that looks like log text — `TLM1` is fine).
- Never send raw binary without a magic/type prefix.
- Throttle or prioritize: telemetry thread retries on `-ENOMEM`, but `Logger` → `ble_send()` does not — sporadic large PID text messages can still starve telemetry.

**RX (write):** Not safe for mixed text+binary on the current firmware. Must switch entirely to envelope dispatch (or keep text-only for RX until a flag day).

**Do not multiplex SMP DFU over NUS** — keep SMP separate.

A second custom characteristic is optional polish (isolate high-rate telemetry from status), not required for correctness if magic+type dispatch and backpressure are done right.

---

## Important Pitfalls Tied to Repository Files

1. **`ble_logger_service.cpp` null-terminated RX parser** — blocks any binary host→robot command with embedded zero bytes; also logs binary as `%.*s`.

2. **`ble_send_binary()` lacks payload bounds checking** — unlike the chat proposal; oversized frames after MTU negotiation failure could misbehave (`/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/bluetooth/src/ble_logger_service.cpp`).

3. **Struct casting on TX** — `telemetry.cpp` sends `&frame` directly; works on nRF52840 but is the wrong pattern for a formal protocol (`telemetry.h`, `telemetry.cpp`).

4. **Shared TX funnel** — `Logger` (`logger.h`), telemetry (`telemetry.cpp`), and PID dumps (`robot_controller.cpp`) all call `bt_nus_send` without global prioritization.

5. **Board BLE buffer config** — `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/applications/app/boards/nrf52840dongle_nrf52840.conf` uses 502-byte buffers; chat/build notes recommend 251 for NCS 2.8 + MCUmgr harmony.

6. **DFU + telemetry overlap** — `dfu_ble.cpp` enters DFU without stopping telemetry; SMP upload (`dfu_operations.py`) shares radio buffers with NUS notifications.

7. **Text command fragility** — `parse_nus_data()` iterates a C string byte-by-byte with implicit concatenation semantics; missing `break` after `TRAJECTORY_MANAGER` (`robot_controller.cpp` ~303–309) falls through to `default` (latent bug pattern for any future cases).

8. **RPi client still text-only** — `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/applications/controller_rpi/app/controller_rpi/nus_service.py` has no binary path; a full conversion must update both PC and RPi clients.

9. **Host-side legacy compatibility** — tests in `/home/kogr/ncs/v2.8.0/self-balancing-robot/robot_control_app/ble_worker/test_data_processor.py` explicitly require legacy text parsing; plan a deprecation window.

---

## Pragmatic Migration Order

1. Add **8-byte envelope + explicit LE helpers**; wrap existing telemetry payload (minimal change to semantics).
2. Replace **`nus_data_parser`** with length-based binary dispatch on RX; migrate commands one `msg_type` at a time.
3. Convert **status/PID/battery/trajectory-ACK TX** to typed NOTIFY messages; remove `Logger` → BLE for hot paths.
4. Add **`ble_send_binary()` MTU clamp** using `bt_gatt_get_mtu(my_conn) - 3` from `ble_logger_setup.cpp`.
5. **Pause telemetry on DFU start** in `dfu_ble.cpp`.
6. Add **host command queue** (already sketched in chat) before binary trajectory traffic.

The telemetry half of the proposed conversion is sound and built. The remaining work for **fully structured binary** is almost entirely on **RX parsing, non-telemetry TX messages, explicit serialization, and DFU/telemetry isolation** — not on creating a new BLE service.
</response>
output_path: /home/kogr/.cursor/projects/home-kogr-ncs-v3-3-0-ncs-door-lock-and-access-control/agent-transcripts/62c13a17-28ae-40c4-bb3c-f70bd130928a/subagents/7fc32775-9ce5-4e0e-bbc6-c4e27cba8970.jsonl
</task>
</system_notification>
<user_query>The beginning of the above subagent result is already visible to the user. Perform any follow-up actions (if needed). DO NOT regurgitate or reiterate its result unless asked. If multiple subagents have now completed and none are still running, briefly summarize the findings and conclusions across all of them. Otherwise, if no follow-ups remain, end your response with a brief third-person confirmation that the subagent has completed. Don't repeat the same confirmation every time.</user_query>

<timestamp>Tuesday, Aug 18, 2026, 2:07 PM (UTC+2)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: subagent
status: success
task_id: a0461e81-914e-452e-a37c-9b4cfa8cf02f
title: Map firmware BLE protocol
detail: <user_visible_high_level_summary>
Mapped the full BLE NUS protocol across firmware and host apps: text command prefixes with parsers, binary TLM1 telemetry, DFU/state/PID/trajectory flows, and which `platform_log` calls are application protocol vs diagnostics. Key risks include mixed binary/text on one characteristic, char-by-char parsing bugs, and DFU staying on SMP.
</user_visible_high_level_summary>
<response>
## BLE NUS Protocol Map — `/home/kogr/ncs/v2.8.0/self-balancing-robot`

**Transport:** Nordic NUS (`6e400001-…`), RX writes host→robot, TX notifications robot→host. 
**Entry point:** `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/bluetooth/src/ble_logger_service.cpp` — `nus_data_received()` → `data_selector()` → registered parsers. 
**TX paths:** `ble_send()` (text) and `ble_send_binary()` (telemetry). Both require connection + notifications (`get_con_status()` && `get_notif_status()`).

**Default build** (`Sources/applications/app/prj.conf`): PID enabled, model identification **disabled**, binary telemetry **enabled** (`CONFIG_ROBOT_CONTROL_LOG=y` + `CONFIG_BLUETOOTH_DRV`).

---

### Command prefix table (`ble_commands.h`)

| Prefix | Char | Firmware handler | Notes |
|--------|------|------------------|-------|
| GET_REGULATOR_PARAMS | `g` | `Robot_Controller::parse_nus_data()` | Triggers async PID dump |
| DFU | `d` | `dfu_process_parser_cb()` | Strip `d`, then subcmd |
| STATE_MACHINE | `m` | `Main_State_Machine::parse_nus_commands()` | Strip `m` first |
| TRAJECTORY | `t` | `Trajectory_Manager::parse_trajectory_point()` | |
| DISTANCE_PID | `f` | `PID::parse_nus_parameters()` / setpoint | |
| LINEAR_SPEED_PID | `s` | `PID::parse_nus_parameters()` | |
| BALANCE_PID | `b` | PID or `LQR::parse_nus_parameters()` | |
| ROTATE_PID | `r` | PID / setpoint | |
| WHEEL_PID | `w` | PID (both wheels) | |
| IDENTIFICATION | `i` | `Model_Identification::identification_data_nus_parser_callback()` | Only if `CONFIG_MODEL_IDENTIFICATION_DRV=y` |

**Subcommand chars:** Regulator `s/k/i/d/f`; State `b/s`; Trajectory `r/f/c`; DFU `b/s`.

---

### Host → Robot (RX) — payloads

| Message | Format | Function | Fields |
|---------|--------|----------|--------|
| Skip DFU | `ds` | `dfu_ble.cpp:dfu_process_parser_cb` | Starts app via `control_loop_init()` |
| Start DFU mode | `db` | same | Blocks in DFU; upload via **SMP GATT**, not NUS |
| Start control | `mb` | `main_state_machine.cpp:parse_nus_commands` | `START` when `READY_TO_START`; with ident drv also calls `activate_identification()` |
| Stop control | `ms` | same | `STOP` when `OPERATION` |
| Get all PID | `g` | `robot_controller.cpp:parse_nus_data` | Submits `send_PID_controllers_parameters()` work |
| Distance setpoint | `fs{float}` | `parse_nus_data` | Meters; blocked if trajectory active |
| Rotation setpoint | `rs{float}` | `parse_nus_data` | Degrees → rad; blocked if trajectory active |
| Balance setpoint | `bs{float}` | `parse_nus_data` | Degrees → rad |
| PID gains | `{f\|s\|b\|r\|w}k{Kp}i{Ki}d{Kd}[f{alpha}]` | `pid.cpp:parse_nus_parameters` | Filter alpha optional |
| LQR gains | `{b}x{Kx}y{Ky}` | `lqr.cpp:parse_nus_parameters` | When `CONFIG_PID_ENABLED=n` |
| Trajectory point | `tr{deg}f{dist}` | `trajectory_manager.cpp:parse_trajectory_point` | Rotation deg→rad, distance m |
| Ident profile | `ip{pwm0_…_pwm9}t{t0_…_t9}` | `model_identification.cpp:identification_data_nus_parser_callback` | 10 PWM values + 10 durations |

**Host senders:** 
- PyQt: `/home/kogr/ncs/v2.8.0/self-balancing-robot/robot_control_app/ble_commands.py`, `right_panel_widget.py` 
- RPi web: `app_dashboard.html`, `motion.html` (via `nus_service.py:send()`)

**Parser quirk:** `parse_nus_data()` iterates **every character** as a potential prefix key. After handling `t…`, remaining chars (`r`, `f`, …) are re-parsed as separate commands — concatenated payloads are unsafe.

---

### Robot → Host (TX) — responses

#### 1. High-rate telemetry (binary — primary path)
**File:** `Sources/robot_control/src/telemetry.cpp` → `ble_send_binary()` 
**Trigger:** `Robot_Controller::normal_motors_control()` → `telemetry_submit()` (unless trajectory `stop_logs()`)

| Field | Type | Notes |
|-------|------|-------|
| magic | u32 | `0x314D4C54` ("TLM1") |
| sequence | u32 | Frame counter |
| dropped_samples | u32 | Queue overflow count |
| sample_count | u16 | Fixed 5 |
| reserved | u16 | 0 |
| samples[5] | 44 B each | See `telemetry.h` |

**Per sample (`Telemetry_Sample`, 44 B):** 
`timestamp_us`, `balance_setpoint`, `balance_angle`, `rotation_setpoint`, `rotation_angle`, `target_speed_0/1`, `measured_speed_0/1`, `pwm_0/1` (angles in degrees).

**Frame size:** 236 B. Queue: 128 samples, `-ENOMEM` retry with 1 ms sleep.

**Host decoder:** `robot_control_app/ble_worker/data_processor.py:process_binary()`

#### 2. Legacy text telemetry (fallback)
**File:** `robot_controller.cpp` — `platform_log()` when binary path not compiled 
**Format:** `bs: %f, ab: %f, rs: %f, ar: %f, ts0: %f, ts1: %f, s0: %f, s1: %f, pwm0: %f, pwm1: %f` 
**Period:** `CONFIG_ROBOT_CONTROL_LOG_NUS_PERIOD_MS` (default 1000 ms)

#### 3. PID parameter response (application protocol)
**File:** `robot_controller.cpp:send_PID_controllers_parameters()` 
**Format:** `Kp{fp}_Ki{fp}_Kd{fp}_` × 5 (distance, linear, balance, rotate, wheel) + `\n` 
**Host parser:** `data_processor.py:__parse_pid_parameters()`

#### 4. Trajectory ACK (application protocol)
**File:** `trajectory_manager.cpp:acknowledge_trajectory_completed()` 
**Payload:** `tc` (2 ASCII bytes) 
**Host:** `nus_service.py` watches `"tc"`; RPi `/nus/wait_ack`

#### 5. Battery (application protocol, boot/DFU only)
**File:** `dfu_ble.cpp:new_battery_level_callback()` 
**Payloads (two messages):** `bat lvl {percent}`, `bat lvl mv {millivolts}` 
**When:** Periodic every 9 s during DFU wait (`!CONFIG_MODEL_IDENTIFICATION_DRV`). **Not sent during normal operation.** 
**Host parser:** `data_processor.py:__parse_battery()`

#### 6. App version (application protocol, DFU start)
**File:** `app_version.cpp:get_app_version_work_handler()` 
**Payload:** `App version: {major}.{minor}.{revision}+{build}`

#### 7. Model identification
- **Command:** NUS `ip…t…` (see above)
- **Data stream:** UART binary frames (`IdentificationFrame`, magic `0xDEADBEEF`, 28 B) from `model_identification.cpp:identification_logger_thread()` — **not BLE**
- **Stop notice:** `Identification stop` via `platform_log` when sequence completes

---

### DFU architecture (preserve separately)
| Layer | Mechanism |
|-------|-----------|
| Mode select | NUS text: `ds` / `db` |
| Firmware transfer | MCUmgr SMP over dedicated BLE service (`SMP_BT_SVC_UUID`), `smpmgr` on host |
| Progress | Internal `LOG_INF` in `dfu_ble.cpp:upload_confirm_handler` — **not sent over NUS** |

PyQt DFU disconnects NUS and uses SMP directly (`dfu_worker/dfu_gui.py`).

---

### `platform_log` → BLE classification

**Mechanism** (`logger.h`): If `CONFIG_BLUETOOTH_DRV && !CONFIG_MODEL_IDENTIFICATION_DRV`, every enabled-module `platform_log()` calls `ble_send(log_msg)` — **raw body only** (no `[ERR] MODULE:` prefix; that goes to UART). Max 244 chars, silent truncation.

#### Application protocol (host should parse)
| File | Message |
|------|---------|
| `telemetry.cpp` | Binary TLM1 frames via `ble_send_binary()` (not `platform_log`) |
| `robot_controller.cpp` | Legacy telemetry string (fallback only) |
| `robot_controller.cpp` | `Kp…_Ki…_Kd…` PID dump |
| `trajectory_manager.cpp` | `tc` |
| `dfu_ble.cpp` | `bat lvl %u`, `bat lvl mv %u` |
| `app_version.cpp` | `App version: …` |
| `model_identification.cpp` | `Identification stop` (boundary/diagnostic) |

#### Diagnostics only (must not be parsed as protocol)
All other `platform_log` calls when their module log Kconfig is enabled:

| Module | File | Examples |
|--------|------|----------|
| APP | `main.cpp` | "Application started." |
| ROBOT_CONTROL | `control_loop.cpp` | "Robot control init finished" |
| TRAJECTORY | (errors only if added) | — |
| MOTOR_CONTROLLER | `motor_controller.cpp` | init/errors |
| IMU | `imu.cpp`, `imu_config.cpp` | angle, gyro, I2C errors |
| ENCODER | `encoder.cpp` | init/errors |
| INTERFACE | `interface.cpp` | LED errors |
| BATTERY_CONTROLLER | `battery_level.cpp` | ADC errors (not level data) |
| UTILS | `utils.cpp` | work reschedule errors |
| WATCHDOG | `watchdog_controller.cpp` | WDG errors |

With default `prj.conf`, most driver logs are **disabled** (`CONFIG_*_LOG=n`), so BLE TX is mostly telemetry + protocol responses. Enabling driver logs will flood NUS.

---

### Recommended typed binary protocol migration

**Goal:** Single demux rule on TX, typed structs everywhere, keep SMP DFU and TLM1 throughput.

```
[ver:u8][type:u8][len:u16][payload…][crc16 optional]
```

| Type | Direction | Replaces |
|------|-----------|----------|
| `0x01` TELEMETRY_FRAME | TX | Current 236 B TLM1 batch (keep layout as payload) |
| `0x10` CMD_SET_PID | RX | `fk…i…d…` text |
| `0x11` CMD_SET_SETPOINT | RX | `fs/rs/bs…` |
| `0x12` CMD_TRAJECTORY | RX | `tr…f…` |
| `0x13` CMD_STATE | RX | `mb/ms` |
| `0x14` CMD_GET_CONFIG | RX | `g` |
| `0x15` CMD_IDENT_PROFILE | RX | `ip…t…` |
| `0x16` CMD_DFU_MODE | RX | `ds/db` (boot only) |
| `0x20` RSP_CONFIG | TX | Kp/Ki/Kd dump |
| `0x21` RSP_TRAJECTORY_DONE | TX | `tc` |
| `0x22` RSP_BATTERY | TX | percent + mV in one frame |
| `0x23` RSP_VERSION | TX | semver struct |
| `0x30` LOG_DIAG | TX | Rate-limited diagnostics (optional, off by default) |
| `0x31` RSP_IDENT_SAMPLE | TX | Move UART ident frames to BLE when ident enabled |

**Preserve unchanged:**
- SMP GATT service + `smpmgr` for image upload
- Telemetry queue/thread/`ENOMEM` retry pattern
- Separate boot-phase handler for `CMD_DFU_MODE`

**Migration phases:**
1. Add envelope + binary RX parser alongside text (`data_selector` checks `payload[0] < 0x20` or dedicated sync byte).
2. Move all TX responses to typed frames; keep text parsers on host for one release.
3. Disable `platform_log` → BLE entirely; route diagnostics to `LOG_DIAG` or UART only.
4. Remove char-by-char text parser.

---

### Risks

1. **Mixed binary/text on one NUS TX char** — Host must check TLM1 magic first; text can collide (e.g. PID dump starting with `Kp`).
2. **Char-by-char RX parsing** — `parse_nus_data()` re-processes suffix chars; `trajectory` case missing `break` falls through to `default`.
3. **RPi motion bug** — `motion.html` sends `rs{deg}fs{dist}` **without `t` prefix**; hits setpoint path, not trajectory manager.
4. **No command ACKs** except trajectory `tc`; silent drops on `-ENOTCONN`/`-ENOMEM`.
5. **Battery only at DFU boot** — GUI battery widget won't update during normal run.
6. **Diagnostic flood** — Any `CONFIG_*_LOG=y` sends unstructured text over NUS.
7. **244-char truncation** — Long PID strings or logs can truncate silently.
8. **Setpoint/PID blocked during trajectory** — Host must wait for `tc`.
9. **`GET_REGULATOR_PARAMS` single-flight** — `m_regulator_message_sending_in_progress` drops rapid repeats.
10. **Ident mode disables BLE logging** — `logger.h` skips `ble_send` when `CONFIG_MODEL_IDENTIFICATION_DRV=y`.
11. **Host newline** — PyQt appends `\n` on send; firmware null-terminates but doesn't strip `\n` before numeric parse (usually harmless).
12. **DFU must stay on SMP** — Do not route firmware bytes through NUS; only mode selection belongs there.

---

### Key file index

| Role | Path |
|------|------|
| Prefix definitions | `Sources/drivers/bluetooth/include/ble_commands.h` |
| NUS RX demux | `Sources/drivers/bluetooth/src/ble_logger_service.cpp` |
| Control/PID/trajectory RX | `Sources/robot_control/src/robot_controller.cpp` |
| PID parse | `Sources/robot_control/src/pid.cpp` |
| Trajectory RX/ACK TX | `Sources/robot_control/src/trajectory_manager.cpp` |
| State RX | `Sources/robot_control/src/main_state_machine.cpp` |
| Binary telemetry TX | `Sources/robot_control/src/telemetry.cpp`, `telemetry.h` |
| DFU mode RX | `Sources/drivers/dfu_smp/src/dfu_ble.cpp` |
| Ident RX | `Sources/drivers/model_identification/src/model_identification.cpp` |
| Logger→BLE gate | `Sources/drivers/logger/include/logger.h` |
| Host binary decode | `robot_control_app/ble_worker/data_processor.py` |
| Host BLE I/O | `robot_control_app/ble_worker/ble_worker.py` |
</response>
output_path: /home/kogr/.cursor/projects/home-kogr-ncs-v3-3-0-ncs-door-lock-and-access-control/agent-transcripts/62c13a17-28ae-40c4-bb3c-f70bd130928a/subagents/a0461e81-914e-452e-a37c-9b4cfa8cf02f.jsonl
</task>
</system_notification>
<user_query>The beginning of the above subagent result is already visible to the user. Perform any follow-up actions (if needed). DO NOT regurgitate or reiterate its result unless asked. If multiple subagents have now completed and none are still running, briefly summarize the findings and conclusions across all of them. Otherwise, if no follow-ups remain, end your response with a brief third-person confirmation that the subagent has completed. Don't repeat the same confirmation every time.</user_query>

<timestamp>Tuesday, Aug 18, 2026, 2:07 PM (UTC+2)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: subagent
status: success
task_id: e983b2c5-197a-4e9e-995c-a81b7f3f9400
title: Map Python BLE protocol
detail: <user_visible_high_level_summary>
Mapped all Python GUI and Raspberry Pi BLE NUS paths, payload formats, and shared binary-envelope recommendations with compatibility concerns.
</user_visible_high_level_summary>
<response>
# BLE NUS Map — self-balancing-robot

## Transport

| Role | UUID |
|------|------|
| Host → Robot (write) | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` |
| Robot → Host (notify) | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` |

**Canonical command alphabet:** `/home/kogr/ncs/v2.8.0/self-balancing-robot/Sources/drivers/bluetooth/include/ble_commands.h` 
**Partial Python mirror:** `/home/kogr/ncs/v2.8.0/self-balancing-robot/robot_control_app/ble_commands.py` (7 constants only)

**Firmware ingress:** `ble_logger_service.cpp` → `data_selector()` → registered callbacks 
**Firmware egress:** `ble_send()` (UTF-8 text) / `ble_send_binary()` (telemetry)

---

## Component Map

### PyQt GUI (`robot_control_app/`)

| File | Role |
|------|------|
| `robot_control_app.py` | Mediator; sends `ds`, `g` on DFU skip; wires signals |
| `ble_worker/ble_worker.py` | Scan/connect/notify/send; binary-vs-text fork on `TLM1` magic |
| `ble_worker/data_processor.py` | Parses text telemetry, battery, PID; binary telemetry frames |
| `widgets/left_panel_widget.py` | Connection UI, battery LED, log console, manual command box |
| `widgets/right_panel_widget.py` | Start/stop, setpoints, PID grid, pyqtgraph plots |
| `ble_commands.py` | Command string constants |
| `dfu_worker/dfu_worker.py` | SMP DFU via `smpmgr` — **not NUS** (disconnects BLE first) |

**Send path:** `RightPanel/LeftPanel` → `send_command()` → `BLEWorker._async_send()` → appends `\n`, `write_gatt_char(..., response=True)` 
**Receive path:** `_notification_handler()` → `DataProcessor.process()` or `process_binary()`

### Raspberry Pi (`Sources/applications/controller_rpi/app/controller_rpi/`)

| File | Role |
|------|------|
| `nus_service.py` | Shared `NUSClient`: connect, notify, send (no `\n`), callbacks |
| `web_server.py` | Flask REST wrapper; log buffer; trajectory-ack event (unused by UI) |
| `templates/app_dashboard.html` | Connect, DFU skip, start/stop, PID get/set, charts from text logs |
| `templates/motion.html` | Phone tilt → streams rotate/distance **setpoints** |
| `check_battery_level.py` | Standalone GPIO blink on low battery via NUS |
| `dfu_ble.py` | SMP/newtmgr DFU — **not NUS** |
| `collect_identification_data_UART.py` | UART binary ident capture — **not NUS** |

---

## Host → Robot Commands (text)

| Payload | Builder | Firmware handler |
|---------|---------|------------------|
| `ds` | GUI skip DFU; RPi `startAppBtn` | `dfu_ble.cpp::dfu_process_parser_cb` → skip, start app |
| `db` | *(no client sends)* | Same → enter SMP DFU mode |
| `mb` | `right_panel_widget`, RPi `enableBtn` | `main_state_machine.cpp::parse_nus_commands("b")` |
| `ms` | `right_panel_widget`, RPi `disableBtn` | Same → `"s"` |
| `g` | GUI after skip; RPi sends `fgsgrgbgwg` | `robot_controller.cpp` → `send_PID_controllers_parameters()` |
| `fs{float}` | GUI distance setpoint (`%.2f`) | Distance setpoint (m) |
| `rs{float}` | GUI rotation setpoint (`%.2f`, deg) | Rotation ramp target (deg→rad) |
| `bs{float}` | *(no GUI button)* | Balance setpoint (deg→rad) |
| `{f\|s\|b\|r\|w}k{kp}i{ki}d{kd}` | GUI PID set (`%.4f`); RPi dashboard | `pid.cpp::parse_nus_parameters` |
| `{f\|s\|b\|r\|w}f{alpha}` | *(no client)* | PID filter alpha |
| `tr{deg}f{meters}` | *(no client)* | `trajectory_manager.cpp::parse_trajectory_point` |
| `ip{pwm1_..._pwm10}t{t1_..._t10}` | RPi ident modal | `model_identification.cpp::identification_data_nus_parser_callback` |
| `rs{deg}fs{m}` | RPi `motion.html` | **Direct setpoints**, not trajectory manager |

**Prefix reference (`ble_commands.h`):** `d`=DFU, `m`=state machine, `t`=trajectory, `f/s/b/r/w`=PIDs, `i`=identification, `g`=get all PID params.

---

## Robot → Host Notifications

### 1. Binary telemetry (PyQt only today)

**Magic:** `TLM1` / `0x314D4C54` 
**Sender:** `telemetry.cpp` → `ble_send_binary()` (236 B, 5 samples/frame) 
**Parser:** `data_processor.py::process_binary()`

```
Header (16 B, LE): magic u32 | sequence u32 | dropped u32 | count u16 | reserved u16
Sample (44 B): timestamp_us u32 | 10× float32
```

| Field | Key |
|-------|-----|
| balance setpoint | `bs` |
| balance angle | `ab` |
| rotation setpoint | `rs` |
| rotation angle | `ar` |
| target speed 0/1 | `ts0`, `ts1` |
| measured speed 0/1 | `s0`, `s1` |
| PWM 0/1 | `pwm0`, `pwm1` |

Enabled when `CONFIG_ROBOT_CONTROL_LOG=y` && `CONFIG_BLUETOOTH_DRV=y` (current `prj.conf`).

### 2. Legacy text telemetry

**Sender:** `robot_controller.cpp` (fallback if binary path off) 
**Format:** `bs: %f, ab: %f, rs: %f, ar: %f, ts0: %f, ts1: %f, s0: %f, s1: %f, pwm0: %f, pwm1: %f` 
**Parser:** `data_processor.py::__parse_telemetry()` — regex `key: float`; RPi `app_dashboard.html::parseLogLine()`

### 3. PID parameters response

**Sender:** `robot_controller.cpp::send_PID_controllers_parameters()` 
**Format:** `Kp%f_Ki%f_Kd%f_` × 5 (distance, linear_speed, balance, rotate, wheel) + `\n` 
**Parser:** GUI `__parse_pid_parameters()`; RPi `parseRobotRegulators()`

### 4. Battery

**Sender:** `dfu_ble.cpp` boot callback (9 s interval) 
**Formats:** `bat lvl %u` (percent), `bat lvl mv %u` (mV) 
**Parser:** GUI `__parse_battery()` (`bat lvl mv`); RPi battery chart (`bat lvl %` only); `check_battery_level.py` (`bat lvl mv`)

### 5. Trajectory ACK

**Sender:** `trajectory_manager.cpp::acknowledge_trajectory_completed()` 
**Format:** `tc` (two chars) 
**Parser:** `nus_service.py` — `"tc" in text.lower()` → `on_trajectory_ack`; `/nus/wait_ack` endpoint exists but **no UI calls it**

### 6. General logs

**Sender:** `logger.h::platform_log()` → `ble_send(log_msg)` (message only, no module prefix over BLE) 
Examples: `Robot control init finished`, `App version: M.m.r+B`, `Identification stop`, driver errors

### 7. Identification data

**Over NUS:** command config only (`ip...t...`) 
**Over UART (not NUS):** binary frames `0xDEADBEEF + 6×float` — `collect_identification_data_UART.py` 
`nus_service.py::IDENT_LINE_RE` for `dt= angle= ...` appears **dead** (firmware never sends that text over BLE)

---

## Feature Coverage by Client

| Feature | PyQt GUI | RPi Web | RPi scripts |
|---------|----------|---------|-------------|
| Connect/notify | ✓ | ✓ | ✓ |
| DFU skip (`ds`) | ✓ | ✓ | — |
| DFU transfer | SMP (no NUS) | SMP (no NUS) | — |
| Start/stop | ✓ | ✓ | — |
| PID get/set | ✓ (`g`) | ✓ (`fgsgrgbgwg` / per-controller) | — |
| Setpoints | ✓ | ✓ (motion) | — |
| Trajectory manager (`tr…f…`, `tc`) | ✗ | infra only | ✗ |
| Binary telemetry | ✓ | ✗ | ✗ |
| Text telemetry charts | ✓ (if binary off) | ✓ | — |
| Battery | mV (GUI) | % (chart) | mV (GPIO) |
| CSV recording | ✓ auto-record | ✗ | ident UART only |
| Ident command | ✗ | ✓ | UART collector |

---

## Tests

Only `/home/kogr/ncs/v2.8.0/self-balancing-robot/robot_control_app/ble_worker/test_data_processor.py`:
- Binary frame parse + sequence metadata
- Rejects short frames
- Legacy text telemetry `bs: 1.5, ab: -2.5`

No tests for command builders, battery/PID regex, RPi clients, or firmware parsers.

---

## Recommended Shared Typed Binary Envelope

Replace ad-hoc text + single-purpose binary telemetry with one framing layer:

```
[ magic u32 = 0x52425431 "RBT1" ]
[ version u8 = 1 ]
[ msg_type u8 ] // enum: CMD, TELEMETRY, PID_STATE, BATTERY, TRAJECTORY_ACK, LOG, IDENT_CFG
[ flags u8 ] // ACK_REQ, FRAGMENT, etc.
[ payload_len u16 ]
[ sequence u32 ] // host→robot or robot→host stream
[ payload ... ] // typed struct per msg_type
[ crc16 optional ]
```

**Typed payloads (examples):**
- `CMD`: `{prefix u8, sub u8, float32 value}` or nested TLV for PID triples
- `TELEMETRY`: reuse current `Telemetry_Sample` batch (already defined in `telemetry.h`)
- `PID_STATE`: `{5 × {kp, ki, kd}}` fixed layout — eliminates fragile `Kp…_Ki…` regex
- `BATTERY`: `{mv u16, percent u8}`
- `TRAJECTORY`: `{rotation_deg f32, distance_m f32}` command; `{status u8}` ack
- `LOG`: `{level u8, len u8, utf8[]}`

**Shared codegen:** generate `ble_commands.h`, Python dataclasses, and JS constants from one schema (e.g. `.yaml` + script).

---

## Compatibility & Testing Concerns

1. **Dual telemetry paths** — Production firmware sends binary; RPi web parses text only → charts silently empty on current build.
2. **Newline inconsistency** — GUI appends `\n`; RPi does not. Firmware null-terminates but does not require `\n`; harmless today, risky if parsers tighten.
3. **Command string drift** — Python `ble_commands.py` incomplete; RPi dashboard hardcodes strings (`fgsgrgbgwg` vs `g`); motion uses setpoints not trajectory API.
4. **Regex fragility** — PID/battery/telemetry parsing breaks on locale (`%f` formatting), log interleaving, or partial BLE notifications.
5. **Prefix collisions** — Single-char routing (`s` = linear-speed PID prefix AND setpoint sub-key) relies on position/context; binary TLV removes ambiguity.
6. **BLE MTU** — Text logs up to 244 B; binary telemetry 236 B/frame; envelope must handle fragmentation or cap payload size (251 B NUS limit in firmware).
7. **DFU mode** — After `ds`, NUS text resumes; SMP uses separate GATT service — envelope must not conflate with SMP transport.
8. **Ident build variant** — `CONFIG_MODEL_IDENTIFICATION_DRV=n` in default `prj.conf`; ident command/UART path untested in default firmware.
9. **Firmware bug** — `robot_controller.cpp` missing `break` after `TRAJECTORY_MANAGER` case (falls through to `default`).
10. **Migration strategy** — Support envelope magic alongside `TLM1` and text logs during transition; version field + feature flags; extend `test_data_processor.py` with round-trip fixtures for every msg_type.

---

## Highest-Impact Fixes (before envelope rollout)

1. Unify command constants in one shared module consumed by PyQt, RPi JS, and docs.
2. Add binary telemetry parsing to RPi (`nus_service.py` or dashboard) — already implemented in PyQt.
3. Replace `fgsgrgbgwg` with `g`; align RPi PID set format to `%.4f` like GUI.
4. Decide trajectory API: either wire `motion.html` to `tr{deg}f{dist}` + `/nus/wait_ack`, or remove dead trajectory ack code.
5. Expand tests: command builders, multi-notification reassembly, battery/PID edge cases, sequence-gap handling.
</response>
output_path: /home/kogr/.cursor/projects/home-kogr-ncs-v3-3-0-ncs-door-lock-and-access-control/agent-transcripts/62c13a17-28ae-40c4-bb3c-f70bd130928a/subagents/e983b2c5-197a-4e9e-995c-a81b7f3f9400.jsonl
</task>
</system_notification>
<user_query>The beginning of the above subagent result is already visible to the user. Perform any follow-up actions (if needed). DO NOT regurgitate or reiterate its result unless asked. If multiple subagents have now completed and none are still running, briefly summarize the findings and conclusions across all of them. Otherwise, if no follow-ups remain, end your response with a brief third-person confirmation that the subagent has completed. Don't repeat the same confirmation every time.</user_query>

Implement the plan as specified, it is attached for your reference. Do NOT edit the plan file itself.

To-do's from the plan have already been created. Do not create them again. Mark them as in_progress as you work, starting with the first one. Don't stop until you have completed all the to-dos.

---

**Cursor**

The shared envelope and firmware dispatch are now in place. I’ve kept DFU image transfer on SMP, converted NUS mode selection and status traffic to typed packets, and separated diagnostic LOG packets from application responses.

The desktop path is now binary-only as well: commands are typed packets, notifications have strict payload decoders, CSV recording opens/closes explicitly, and GUI plotting remains throttled while every sample is recorded. I also changed DFU skip acknowledgment so the GUI only unlocks after firmware initialization is actually complete.

Implemented all Binary-Only BLE Protocol plan items without modifying the plan file.

Key results:
- Unified `RBT1` little-endian protocol and typed messages.
- Binary telemetry, commands, PID/LQR, battery, version, DFU selection, trajectory, identification, results, and UTF-8 LOG packets.
- Prioritized serialized NUS TX queues with MTU validation.
- Binary-only Python GUI with queued writes and strict decoding.
- Explicit CSV lifecycle; every telemetry sample recorded.
- Legacy command definitions removed.
- SMP firmware transfer remains separate.

Verification:
- Default firmware build passed.
- Model-identification build passed.
- LQR build passed.
- 15 Python tests passed.
- Python compilation and lint checks passed.
- Generated DFU package successfully.
- Hardware BLE/DFU testing was not performed.

---

**User**

<timestamp>Tuesday, Aug 18, 2026, 2:42 PM (UTC+2)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: subagent
status: success
task_id: aa8a8c0b-93fc-41df-be4c-12c913953d05
title: Migrate desktop binary protocol
detail: <user_visible_high_level_summary>
Implemented strict binary RBT1 desktop BLE commands and notification decoding, a serialized acknowledged write queue, typed telemetry/status handling, and explicit CSV recording lifecycle. Added protocol, decoder, queue, malformed-packet, and recording tests; all 12 tests and compileall pass.
</user_visible_high_level_summary>
<response>
Implemented desktop-only binary BLE support. Firmware/C++, plan, DFU/SMP, and `ble_protocol.py` were untouched.

Changed:
- `robot_control_app/ble_worker/ble_worker.py`
- `robot_control_app/ble_worker/data_processor.py`
- `robot_control_app/robot_control_app.py`
- `robot_control_app/widgets/right_panel_widget.py`
- `robot_control_app/widgets/left_panel_widget.py`
- `robot_control_app/ble_worker/test_data_processor.py`
- `robot_control_app/ble_worker/test_ble_worker.py`
- `robot_control_app/test_ble_protocol.py`

Validation:
- 12 unit tests passed
- `compileall` passed
- No lint or whitespace errors
- No commit created
</response>
output_path: /home/kogr/.cursor/projects/home-kogr-ncs-v3-3-0-ncs-door-lock-and-access-control/agent-transcripts/62c13a17-28ae-40c4-bb3c-f70bd130928a/subagents/aa8a8c0b-93fc-41df-be4c-12c913953d05.jsonl
</task>
</system_notification>
<user_query>The beginning of the above subagent result is already visible to the user. Perform any follow-up actions (if needed). DO NOT regurgitate or reiterate its result unless asked. If multiple subagents have now completed and none are still running, briefly summarize the findings and conclusions across all of them. Otherwise, if no follow-ups remain, end your response with a brief third-person confirmation that the subagent has completed. Don't repeat the same confirmation every time.</user_query>

---

**Cursor**

No follow-up needed. The subagent has completed successfully.

