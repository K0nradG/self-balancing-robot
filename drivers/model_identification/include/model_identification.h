#ifndef MODEL_IDENTIFICATION_H
#define MODEL_IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

#define SAMPLE_RATE_MS 1
#define RECORD_TIME_MS 10000
#define BUFFER_SIZE (RECORD_TIME_MS / SAMPLE_RATE_MS)

#define BUFFER_COUNT \
    4  // Independent buffers number. One buffer is 4kB (1000 floats), 4 buffers - 16kB
#define ANGLE_BUFFER_ID 0
#define ANGLE_DT_BUFFER_ID 1
#define U_BUFFER_ID 2
#define TIME_BUFFER_ID 3

void
buffer_init(void);

bool
buffer_put(uint8_t buffer_id, float data);

uint16_t
buffer_get(uint8_t buffer_id, float* data, uint16_t max_len);

bool
buffer_all_full(void);

#endif /* MODEL_IDENTIFICATION_H */
