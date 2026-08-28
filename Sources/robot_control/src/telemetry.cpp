// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "telemetry.h"
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "ble_connection.h"
#include "ble_protocol.h"
#include "ble_protocol_constants.h"
#include "ble_service.h"

namespace Robot_Control
{
namespace
{

constexpr size_t max_samples_per_frame      = 5u;
constexpr size_t telemetry_queue_length     = 128u;
constexpr size_t telemetry_metadata_size    = 8u;
constexpr size_t telemetry_sample_size      = 44u;
constexpr size_t telemetry_payload_capacity = telemetry_metadata_size + (max_samples_per_frame * telemetry_sample_size);
constexpr int send_retry_delay_ms           = 1;
constexpr size_t telemetry_thread_stack_size      = 2048u;
constexpr int telemetry_thread_priority           = 5;
constexpr uint32_t telemetry_thread_options       = 0u;
constexpr int32_t telemetry_thread_start_delay_ms = 0;
static_assert(telemetry_payload_capacity == 228u);

K_MSGQ_DEFINE(telemetry_queue, sizeof(Telemetry_Sample), telemetry_queue_length, alignof(Telemetry_Sample));

atomic_t dropped_samples;

// Choose the largest number of samples that fits in one BLE packet:
//   ATT payload >= 240 B: 5 samples, 100 packets/s
//   ATT payload >= 196 B: 4 samples, 125 packets/s
//   ATT payload >= 152 B: 3 samples, ~167 packets/s
//   ATT payload >= 108 B: 2 samples, 250 packets/s
//   ATT payload >=  64 B: 1 sample,  500 packets/s
//   ATT payload <   64 B: no complete sample fits
size_t
get_samples_per_frame(uint16_t att_payload_size)
{
    size_t const fixed_size = BLE_Protocol::HEADER_SIZE + telemetry_metadata_size;
    if(att_payload_size < (fixed_size + telemetry_sample_size))
    {
        return 0u;
    }

    size_t const sample_capacity = (att_payload_size - fixed_size) / telemetry_sample_size;
    if(sample_capacity > max_samples_per_frame)
    {
        return max_samples_per_frame;
    }

    return sample_capacity;
}

void
write_sample(BLE_Protocol::Payload_Writer& payload_writer, Telemetry_Sample const& sample)
{
    payload_writer.put_u32(sample.timestamp_us);
    payload_writer.put_float(sample.balance_setpoint);
    payload_writer.put_float(sample.balance_angle);
    payload_writer.put_float(sample.rotation_setpoint);
    payload_writer.put_float(sample.rotation_angle);
    payload_writer.put_float(sample.target_speed_0);
    payload_writer.put_float(sample.target_speed_1);
    payload_writer.put_float(sample.measured_speed_0);
    payload_writer.put_float(sample.measured_speed_1);
    payload_writer.put_float(sample.pwm_0);
    payload_writer.put_float(sample.pwm_1);
}

void
wait_for_first_sample(Telemetry_Sample* samples)
{
    k_msgq_get(&telemetry_queue, &samples[0], K_FOREVER);
}

void
wait_for_other_samples(Telemetry_Sample* samples, size_t sample_count)
{
    for(size_t i = 1u; i < sample_count; ++i)
    {
        k_msgq_get(&telemetry_queue, &samples[i], K_FOREVER);
    }
}

void
write_telemetry_payload(
    BLE_Protocol::Payload_Writer& payload_writer, Telemetry_Sample const* samples, size_t sample_count)
{
    payload_writer.put_u32(static_cast<uint32_t>(atomic_get(&dropped_samples)));
    payload_writer.put_u8(static_cast<uint8_t>(sample_count));
    payload_writer.put_u8(0u);
    payload_writer.put_u8(0u);
    payload_writer.put_u8(0u);

    for(size_t i = 0u; i < sample_count; ++i)
    {
        write_sample(payload_writer, samples[i]);
    }
}

int
send_telemetry_payload(BLE_Protocol::Payload_Writer const& payload_writer)
{
    int send_result;
    do
    {
        send_result = ble_send_telemetry_packet(payload_writer);
        if(send_result == -ENOMEM)
        {
            k_sleep(K_MSEC(send_retry_delay_ms));
        }
    } while(send_result == -ENOMEM);

    return send_result;
}

void
count_samples_not_sent(int send_result, size_t sample_count)
{
    if((send_result != 0) && (send_result != -ENOTCONN))
    {
        atomic_add(&dropped_samples, sample_count);
    }
}

void
telemetry_thread(void*, void*, void*)
{
    Telemetry_Sample samples[max_samples_per_frame] {};
    uint8_t payload[telemetry_payload_capacity] {};

    while(true)
    {
        wait_for_first_sample(samples);

        size_t const sample_count = get_samples_per_frame(get_att_payload_size());
        if(sample_count == 0u)
        {
            atomic_inc(&dropped_samples);
            continue;
        }

        wait_for_other_samples(samples, sample_count);

        BLE_Protocol::Payload_Writer payload_writer(payload, sizeof(payload));
        write_telemetry_payload(payload_writer, samples, sample_count);

        int const send_result = send_telemetry_payload(payload_writer);
        count_samples_not_sent(send_result, sample_count);
    }
}

K_THREAD_DEFINE(
    telemetry_thread_id, telemetry_thread_stack_size, telemetry_thread, nullptr, nullptr, nullptr,
    telemetry_thread_priority, telemetry_thread_options, telemetry_thread_start_delay_ms);

}  // namespace

void
telemetry_submit(Telemetry_Sample const& sample)
{
    if(k_msgq_put(&telemetry_queue, &sample, K_NO_WAIT) != 0)
    {
        atomic_inc(&dropped_samples);
    }
}

}  // namespace Robot_Control
