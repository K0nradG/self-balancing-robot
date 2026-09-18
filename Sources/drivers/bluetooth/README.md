# Bluetooth protocol

The robot and the Python control application communicate through the Nordic UART Service (NUS) using binary RBT1 packets. All multi-byte values use little-endian byte order.

## Packet format

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | Magic value `RBT1` (`0x31544252`) |
| 4 | 1 | Message type |
| 5 | 1 | Reserved (currently zero) |
| 6 | 2 | Payload length |
| 8 | 4 | Packet number |
| 12 | variable | Message payload |

The header is 12 bytes. A packet contains exactly one header and one payload; RBT1 does not fragment or reassemble packets.

## Communication flow

- The Python application writes command packets to the NUS RX characteristic.
- The robot validates and handles each command, then normally sends a `COMMAND_RESULT` containing the request packet number, request type, and status.
- The robot independently sends telemetry, logs, and state notifications through the NUS TX characteristic.
- Telemetry and logs do not receive application-level acknowledgements.
- Protocol responses have priority over telemetry, which has priority over logs.

## Packet size and ATT MTU

The protocol supports packets up to 244 bytes because a commonly used ATT MTU of 247 bytes leaves `247 - 3 = 244` bytes for a GATT notification value. The 3 bytes are used by the ATT notification header.

The negotiated ATT MTU may be smaller. The firmware starts with the default 20-byte value limit (`ATT MTU 23 - 3`) and updates it after MTU exchange. Telemetry dynamically uses the largest batch of one to five samples that fits: five samples require a 240-byte ATT payload (MTU 243), while one sample requires 64 bytes (MTU 67). Below that limit telemetry cannot be sent because RBT1 does not fragment packets. Other packets larger than the current limit are rejected with `-EMSGSIZE`.

## Processing examples

### Log notification

For example, a robot module requests an `INF` log:

```cpp
robot_control_logger.platform_log(LOG_LEVEL::INF, "sample text");
```

The log follows this call path:

```text
Logger::platform_log("sample text")
    |
    v
vsnprintf() creates the text "sample text"
    |
    v
ble_send_log(INF, "ROBOT_CONTROL", "sample text")
    |
    v
get_log_module_length()
get_log_message_length()
    |
    v
write_log_payload()
    |
    v
put_packet_in_queue(log_tx_queue, LOG, payload)
    |
    v
check_packet_before_queue()
    |
    v
build_tx_packet()
    |
    v
k_msgq_put(log_tx_queue)
    |
    v
k_sem_give(tx_available)
    |
    v
tx_thread() wakes up
    |
    v
get_next_packet()
    |
    v
send_packet_to_nus()
    |
    v
bt_nus_send()
    |
    v
Python application
    |
    v
unpack_packet() checks the RBT1 frame
    |
    v
DataProcessor reads the LOG payload
    |
    v
GUI displays "[1] ROBOT_CONTROL: sample text"
```

The resulting packet is:

```text
RBT1 header:
  type           = LOG (0x02)
  payload_length = 28
  packet_number  = <robot TX packet number>

LOG payload:
  level          = INF (0x01)
  module_length  = 13
  text_length    = 11
  module         = "ROBOT_CONTROL"
  text           = "sample text"
```

The 12-byte header and 28-byte payload form one 40-byte NUS notification. The Python application checks the RBT1 header, reads the LOG payload, decodes both UTF-8 strings, and currently displays `[1] ROBOT_CONTROL: sample text`. Logs do not receive a `COMMAND_RESULT` response.

### Command and response

When the user clicks **Start Control**, the command and its response follow this call path:

```text
User clicks "Start Control"
    |
    v
ble_protocol.state_command(StateAction.START)
    |
    v
ble_protocol.pack_packet(STATE_COMMAND, START)
    |
    v
send_command_requested signal
    |
    v
BLEWorker.send_command()
    |
    v
unpack_packet() checks the command
    |
    v
pack_packet() adds the next app packet number
    |
    v
asyncio write queue
    |
    v
BLEWorker._write_loop()
    |
    v
BleakClient.write_gatt_char(NUS_RX, command, response=True)
    |
    v
Robot: nus_data_received()
    |
    v
BLE_Protocol::decode_packet()
    |
    v
send_received_packet_to_callback()
    |
    v
ble_packet_callback()
    |
    v
Robot_Controller::handle_ble_packet()
    |
    v
Payload_Reader::get_u8() reads START
    |
    v
Main_State_Machine::apply_command(START)
    |
    v
send_command_result()
    |
    v
ble_send_packet(COMMAND_RESULT)
    |
    v
put_packet_in_queue(protocol_tx_queue, COMMAND_RESULT, payload)
    |
    v
tx_thread() -> get_next_packet() -> send_packet_to_nus()
    |
    v
bt_nus_send()
    |
    v
Python: BLEWorker._notification_handler()
    |
    v
unpack_packet() checks the COMMAND_RESULT frame
    |
    v
DataProcessor._parse_command_result()
    |
    v
GUI displays the command status
```

`response=True` requests a GATT write response from BLE. It is separate from the RBT1 `COMMAND_RESULT`, which reports whether the robot accepted and applied the command.

Python sends this command packet:

```text
RBT1 header:
  type           = STATE_COMMAND (0x20)
  payload_length = 1
  packet_number  = 7

Payload:
  action         = START (0x00)
```

The robot decodes the packet, checks whether START is valid in its current state, applies it, and sends:

```text
RBT1 header:
  type           = COMMAND_RESULT (0x07)
  payload_length = 6
  packet_number  = <robot TX packet number>

Payload:
  request_packet_number = 7
  request_type          = STATE_COMMAND (0x20)
  status                = OK (0x00)
```

The response has its own robot TX packet number. Python matches the result to the request using `request_packet_number` inside the payload.

### Telemetry frame with five samples

With an ATT MTU of at least 243, the robot batches five samples collected at 2 ms intervals. The complete call path is:

```text
Robot_Controller::normal_motors_control()
    |
    v
creates one Telemetry_Sample
    |
    v
telemetry_submit(sample)
    |
    v
k_msgq_put(telemetry_queue)
    |
    v
telemetry_thread() receives the first sample
    |
    v
samples_per_frame(get_att_payload_size())
    |
    v
selects 5 samples for a large enough MTU
    |
    v
k_msgq_get() receives the remaining 4 samples
    |
    v
Payload_Writer writes dropped_samples, sample_count and reserved bytes
    |
    v
encode_sample() writes each of the 5 samples
    |
    v
ble_send_telemetry_packet()
    |
    v
put_packet_in_queue(telemetry_tx_queue, TELEMETRY, payload)
    |
    v
check_packet_before_queue()
    |
    v
build_tx_packet()
    |
    v
k_msgq_put(telemetry_tx_queue)
    |
    v
k_sem_give(tx_available)
    |
    v
tx_thread() wakes up
    |
    v
get_next_packet()
    |
    v
send_packet_to_nus()
    |
    v
bt_nus_send()
    |
    v
Python: BLEWorker._notification_handler()
    |
    v
unpack_packet() checks the TELEMETRY frame
    |
    v
DataProcessor._parse_telemetry()
    |
    v
reads sample_count and decodes all 5 samples
    |
    v
all samples can be written to CSV; the latest sample updates the GUI
```

The resulting telemetry packet is:

```text
RBT1 header                         12 bytes
  type           = TELEMETRY (0x01)
  payload_length = 228
  packet_number  = <telemetry packet number>

Telemetry metadata                  8 bytes
  dropped_samples = <total dropped>
  sample_count    = 5
  reserved        = 00 00 00

Five samples                      220 bytes
  each sample:
    timestamp_us                   4 bytes
    balance_setpoint               4 bytes (float)
    balance_angle                  4 bytes (float)
    rotation_setpoint              4 bytes (float)
    rotation_angle                 4 bytes (float)
    target_speed_0/1               8 bytes (2 floats)
    measured_speed_0/1             8 bytes (2 floats)
    pwm_0/1                        8 bytes (2 floats)

Complete RBT1 packet              240 bytes
```

The frame is sent every 10 ms. Python validates `payload_length`, reads `sample_count`, verifies that exactly `5 * 44` sample bytes remain, and decodes every timestamp and float. It then updates the UI and optionally writes all five samples to CSV. Telemetry is unacknowledged; packet-number gaps and `dropped_samples` indicate data loss.

The matching implementations are:

- Firmware: `include/ble_protocol.h` and `src/ble_protocol.cpp`
- Python: `robot_control_app/ble_protocol.py`
