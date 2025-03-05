#ifndef MODEL_IDENTIFICATION_H
#define MODEL_IDENTIFICATION_H

#include <stdint.h>
#include "imu.h"


#ifdef __cplusplus
extern "C" {
#endif

#define RECORD_TIME_MS 10000
#define BUFFER_SIZE (RECORD_TIME_MS / CONFIG_MODEL_IDENTIFICATION_SAMPLE_TIME)

#define BUFFER_COUNT 4  // Independent buffers number. One buffer is 4kB (1000 floats), 4 buffers - 16kB
#define ANGLE_BUFFER_ID 0
#define ANGLE_DT_BUFFER_ID 1
#define U_BUFFER_ID 2
#define TIME_BUFFER_ID 3

void
new_imu_data_for_identification(struct identification_data data);

uint16_t
buffer_get(uint8_t buffer_id, float* data, uint16_t max_len);

void
model_identification_start(void);

void
model_identification_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_IDENTIFICATION_H */
