#include "model_identification.h"
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <cstdint>
#include "control_loop.h"
#include "logger.h"
#include "main_state_machine.h"

namespace Robot_Control
{

static Logger<IS_ENABLED(CONFIG_MODEL_IDENTIFICATION_LOG)> logger("MODEL");

static ring_buf g_buffers[BUFFER_COUNT];
static uint8_t g_buffer_data[BUFFER_COUNT][BUFFER_SIZE * sizeof(float)];
static uint16_t g_buffer_index[BUFFER_COUNT];
static bool g_is_full[BUFFER_COUNT];
static float g_temp_buffer[BUFFER_SIZE];

struct identification_data g_identification_data;

enum class IdentificationState
{
    STOPPED,
    COLLECTING,
    SENDING
};
static IdentificationState g_state = IdentificationState::STOPPED;

static void
identification_data_sending_work_handler(k_work* work);
static void
model_identification_work_handler(k_work* work);

K_WORK_DELAYABLE_DEFINE(model_identification_work, model_identification_work_handler);
K_WORK_DELAYABLE_DEFINE(identification_data_sending_work, identification_data_sending_work_handler);

static void
buffers_reset()
{
    for(uint8_t i = 0; i < BUFFER_COUNT; i++)
    {
        ring_buf_reset(&g_buffers[i]);
        g_buffer_index[i] = 0;
        g_is_full[i]      = false;
    }
}

static bool
buffer_put(uint8_t buffer_id, float data)
{
    if(buffer_id >= BUFFER_COUNT || g_is_full[buffer_id])
        return false;

    int ret = ring_buf_put(&g_buffers[buffer_id], (uint8_t*)&data, sizeof(float));
    if(ret == sizeof(float))
    {
        g_buffer_index[buffer_id]++;
        if(g_buffer_index[buffer_id] >= BUFFER_SIZE)
            g_is_full[buffer_id] = true;
        return true;
    }
    return false;
}

static uint16_t
buffer_get(uint8_t buffer_id, float* data, uint16_t max_len)
{
    if(buffer_id >= BUFFER_COUNT)
        return 0;

    uint16_t len = (g_buffer_index[buffer_id] < max_len) ? g_buffer_index[buffer_id] : max_len;
    for(uint16_t i = 0; i < len; i++)
    {
        ring_buf_get(&g_buffers[buffer_id], (uint8_t*)&data[i], sizeof(float));
    }
    g_buffer_index[buffer_id] -= len;
    return len;
}

static bool
buffer_all_full()
{
    for(uint8_t i = 0; i < BUFFER_COUNT; i++)
    {
        if(!g_is_full[i])
            return false;
    }
    return true;
}

int
identification_init()
{
    for(uint8_t i = 0; i < BUFFER_COUNT; i++)
    {
        ring_buf_init(&g_buffers[i], BUFFER_SIZE * sizeof(float), g_buffer_data[i]);
        g_buffer_index[i] = 0;
        g_is_full[i]      = false;
    }
    return 0;
}

void
new_regulator_data_for_identification(identification_data data)
{
    g_identification_data = data;
}

static void
model_identification_work_handler(k_work* work)
{
    ARG_UNUSED(work);

    if(!buffer_all_full())
    {
        buffer_put(ANGLE_BUFFER_ID, g_identification_data.angle);
        buffer_put(ANGLE_DT_BUFFER_ID, g_identification_data.angle_dt);
        buffer_put(U_BUFFER_ID, g_identification_data.pwm);
        buffer_put(TIME_BUFFER_ID, g_identification_data.dt);
    }
    else if(g_state == IdentificationState::COLLECTING)
    {
        Robot_Control::Main_State_Machine::instance().set_ready_to_start();

        g_state = IdentificationState::SENDING;
        int err = k_work_submit(&identification_data_sending_work.work);
        if(err != 0)
        {
            logger.platform_log(LOG_LEVEL::ERR, "Data sending trigger failed, err: %d", err);
        }
        return;
    }
}

void
trigger_collecting_identification_data()
{
    if(g_state == IdentificationState::STOPPED)
    {
        buffers_reset();
        g_state = IdentificationState::COLLECTING;
    }

    int err = k_work_submit(&model_identification_work.work);
    if(err != 0)
    {
        logger.platform_log(LOG_LEVEL::ERR, "Data collecting trigger failed, err: %d", err);
    }
}

static void
identification_data_sending_work_handler(k_work* work)
{
    ARG_UNUSED(work);
    logger.platform_log(LOG_LEVEL::INF, "Data sending started");

    for(uint8_t i = 0; i < BUFFER_COUNT; i++)
    {
        uint16_t len = buffer_get(i, g_temp_buffer, BUFFER_SIZE);
        if(len > 0)
        {
            for(uint16_t j = 0; j < len; j++)
            {
                char data_str[16];
                snprintf(data_str, sizeof(data_str), "%.3f", static_cast<double>(g_temp_buffer[j]));
                logger.platform_log(LOG_LEVEL::INF, "%s", data_str);
                k_sleep(K_MSEC(20));
            }
            logger.platform_log(LOG_LEVEL::INF, "----------------------------");
        }
    }

    logger.platform_log(LOG_LEVEL::INF, "Data sending finished");
    g_state = IdentificationState::STOPPED;
}

}  // namespace Robot_Control
