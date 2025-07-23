#ifndef MODEL_IDENTIFICATION_H
#define MODEL_IDENTIFICATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RECORD_TIME_MS 10000
#define BUFFER_SIZE (RECORD_TIME_MS / CONFIG_MODEL_IDENTIFICATION_SAMPLE_TIME)

#define BUFFER_COUNT 4u  // Independent buffers number. One buffer is 4kB (1000 floats), 4 buffers - 16kB
#define ANGLE_BUFFER_ID 0u
#define ANGLE_DT_BUFFER_ID 1u
#define U_BUFFER_ID 2u
#define TIME_BUFFER_ID 3u

#if defined(CONFIG_MODEL_IDENTIFICATION_DRV)
typedef struct 
{
    float dt;
    float pwm;
    float angle;
    float angle_dt;
} identification_data;

void
new_regulator_data_for_identification(identification_data data);
#endif // CONFIG_MODEL_IDENTIFICATION_DRV

uint16_t
buffer_get(uint8_t buffer_id, float* data, uint16_t max_len);

void
model_identification_start(void);

void
notify_data_sent(void);

void
trigger_collecting_identification_data();

#ifdef __cplusplus
}
#endif

#endif /* MODEL_IDENTIFICATION_H */
