// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "telemetry.h"
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "ble_protocol.h"
#include "ble_service.h"

namespace Robot_Control
{
namespace
{

constexpr size_t samples_per_frame       = 5u;
constexpr size_t telemetry_queue_length  = 128u;
constexpr size_t telemetry_metadata_size = 8u;
constexpr size_t telemetry_sample_size   = 44u;
constexpr size_t telemetry_payload_size  = telemetry_metadata_size + (samples_per_frame * telemetry_sample_size);
static_assert(telemetry_payload_size == 228u);

K_MSGQ_DEFINE(telemetry_queue, sizeof(Telemetry_Sample), telemetry_queue_length, alignof(Telemetry_Sample));

atomic_t dropped_samples;

void
encode_sample(BLE_Protocol::Payload_Writer& writer, Telemetry_Sample const& sample)
{
    writer.put_u32(sample.timestamp_us);
    writer.put_float(sample.balance_setpoint);
    writer.put_float(sample.balance_angle);
    writer.put_float(sample.rotation_setpoint);
    writer.put_float(sample.rotation_angle);
    writer.put_float(sample.target_speed_0);
    writer.put_float(sample.target_speed_1);
    writer.put_float(sample.measured_speed_0);
    writer.put_float(sample.measured_speed_1);
    writer.put_float(sample.pwm_0);
    writer.put_float(sample.pwm_1);
}

void
telemetry_thread(void*, void*, void*)
{
    Telemetry_Sample samples[samples_per_frame] {};
    uint8_t payload[telemetry_payload_size] {};

    while(true)
    {
        for(size_t i = 0u; i < samples_per_frame; ++i)
        {
            k_msgq_get(&telemetry_queue, &samples[i], K_FOREVER);
        }

        BLE_Protocol::Payload_Writer writer(payload, sizeof(payload));
        writer.put_u32(static_cast<uint32_t>(atomic_get(&dropped_samples)));
        writer.put_u8(static_cast<uint8_t>(samples_per_frame));
        writer.put_u8(0u);
        writer.put_u8(0u);
        writer.put_u8(0u);
        for(size_t i = 0u; i < samples_per_frame; ++i)
        {
            encode_sample(writer, samples[i]);
        }

        int err;
        do
        {
            err = ble_send_telemetry_packet(writer);
            if(err == -ENOMEM)
            {
                k_sleep(K_MSEC(1));
            }
        } while(err == -ENOMEM);

        if((err != 0) && (err != -ENOTCONN))
        {
            atomic_add(&dropped_samples, samples_per_frame);
        }
    }
}

K_THREAD_DEFINE(telemetry_thread_id, 2048, telemetry_thread, nullptr, nullptr, nullptr, 5, 0, 0);

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
