// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "telemetry.h"
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "ble_connection.h"
#include "ble_protocol_constants.h"
#include "ble_service.h"

namespace Robot_Control
{
namespace
{

constexpr size_t MAX_SAMPLES_PER_FRAME      = 5u;
constexpr size_t TELEMETRY_METADATA_SIZE    = 8u;
constexpr size_t TELEMETRY_SAMPLE_SIZE      = 44u;
constexpr size_t TELEMETRY_PAYLOAD_CAPACITY = TELEMETRY_METADATA_SIZE + (MAX_SAMPLES_PER_FRAME * TELEMETRY_SAMPLE_SIZE);
static_assert(TELEMETRY_PAYLOAD_CAPACITY == 228u);

constexpr size_t TELEMETRY_THREAD_STACK_SIZE      = 2048u;
constexpr int TELEMETRY_THREAD_PRIORITY           = 5;
constexpr uint32_t TELEMETRY_THREAD_OPTIONS       = 0u;
constexpr int32_t TELEMETRY_THREAD_START_DELAY_MS = 0;

constexpr size_t TELEMETRY_QUEUE_LENGTH = 128u;
K_MSGQ_DEFINE(telemetry_queue, sizeof(Telemetry_Sample), TELEMETRY_QUEUE_LENGTH, alignof(Telemetry_Sample));

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
    size_t const fixed_size = BLE_Protocol::HEADER_SIZE + TELEMETRY_METADATA_SIZE;
    if(att_payload_size < (fixed_size + TELEMETRY_SAMPLE_SIZE))
    {
        return 0u;
    }

    size_t const sample_capacity = (att_payload_size - fixed_size) / TELEMETRY_SAMPLE_SIZE;
    if(sample_capacity > MAX_SAMPLES_PER_FRAME)
    {
        return MAX_SAMPLES_PER_FRAME;
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
    int send_result {};
    do
    {
        send_result = ble_send_telemetry_packet(payload_writer);
        if(send_result == -ENOMEM)
        {
            k_sleep(K_MSEC(1));
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
    Telemetry_Sample samples[MAX_SAMPLES_PER_FRAME] {};
    uint8_t payload[TELEMETRY_PAYLOAD_CAPACITY] {};

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
    telemetry_thread_id, TELEMETRY_THREAD_STACK_SIZE, telemetry_thread, nullptr, nullptr, nullptr,
    TELEMETRY_THREAD_PRIORITY, TELEMETRY_THREAD_OPTIONS, TELEMETRY_THREAD_START_DELAY_MS);

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
