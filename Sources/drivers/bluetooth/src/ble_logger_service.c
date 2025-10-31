#include <bluetooth/services/nus.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include "ble_commands.h"
#include "ble_connection.h"
#include "ble_service.h"

#define BLE_NUS_MAX_DATA_LEN 251

LOG_MODULE_REGISTER(ble_nus, CONFIG_LOGGER_LOG_LEVEL);

static bool s_nus_notification_enabled                                       = false;
static regulator_parameters_parser_cb_t s_regulator_parameters_parser_cb     = NULL;
static dfu_process_parser_cb_t s_dfu_process_parser_cb                       = NULL;
static state_machine_commands_parser_cb_t s_state_machine_commands_parser_cb = NULL;

bool
get_notif_status()
{
    return s_nus_notification_enabled;
}

void
set_notif_status(bool nus_notification_enabled)
{
    s_nus_notification_enabled = nus_notification_enabled;
}

static void
data_selector(const char* data)
{
    if(data[0] == DFU_PREFIX)
    {
        if(s_dfu_process_parser_cb)
        {
            s_dfu_process_parser_cb(data);
        }
        return;
    }

    if((data[0] == REG_DISTANCE_PID_PREFIX) || (data[0] == REG_LINEAR_SPEED_PID_PREFIX) ||
       (data[0] == REG_BALANCE_PID_PREFIX) || (data[0] == REG_ROTATE_PID_PREFIX) || (data[0] == REG_WHEEL_PID_PREFIX) ||
       (data[0] == TRAJECTORY_MANAGER_PREFIX))
    {
        if(s_regulator_parameters_parser_cb)
        {
            s_regulator_parameters_parser_cb(data);
        }
    }

    if(data[0] == STATE_MACHINE_PREFIX)
    {
        data++;
        if(s_state_machine_commands_parser_cb)
        {
            s_state_machine_commands_parser_cb(data);
        }
    }
}

static void
nus_data_parser(const uint8_t* data, uint16_t len)
{
    if(len > BLE_NUS_MAX_DATA_LEN)
    {
        LOG_ERR("Data length exceeds buffer size!");
        return;
    }

    static char received_data[BLE_NUS_MAX_DATA_LEN + 1];
    memset(received_data, 0, sizeof(received_data));

    memcpy(received_data, data, len);
    received_data[len] = '\0';

    data_selector(received_data);
}

static void
nus_data_received(struct bt_conn* conn, const uint8_t* data, uint16_t len)
{
    nus_data_parser(data, len);
    LOG_INF("NUS received data: %.*s", len, data);
}

void
nus_notif_enabled(enum bt_nus_send_status status)
{
    if(status == BT_NUS_SEND_STATUS_ENABLED)
    {
        set_notif_status(true);
        LOG_INF("notification enabled");
    }
    else if(status == BT_NUS_SEND_STATUS_DISABLED)
    {
        set_notif_status(false);
        LOG_INF("notification disabled");
    }
}

static struct bt_nus_cb nus_callbacks = {
    .send_enabled = nus_notif_enabled,
    .received     = nus_data_received,
};

int
ble_service_init(void)
{
    int const err = bt_nus_init(&nus_callbacks);
    if(err != 0)
    {
        LOG_ERR("NUS initialization error: %d", err);
    }
    return err;
}

void
ble_send(char* data)
{
    if(get_con_status() && get_notif_status())
    {
        int const err = bt_nus_send(NULL, data, strlen(data));
        if(err != 0)
        {
            LOG_ERR("NUS failed to send data: %d", err);
        }
    }
}

void
new_regulator_parameters_parser_cb_register(regulator_parameters_parser_cb_t _regulator_parameters_parser_cb)
{
    if(_regulator_parameters_parser_cb)
    {
        s_regulator_parameters_parser_cb = _regulator_parameters_parser_cb;
    }
}

void
dfu_process_parser_cb_register(dfu_process_parser_cb_t _dfu_process_parser_cb)
{
    if(_dfu_process_parser_cb)
    {
        s_dfu_process_parser_cb = _dfu_process_parser_cb;
    }
}

void
state_machine_commands_parser_cb_register(state_machine_commands_parser_cb_t _state_machine_commands_parser_cb)
{
    if(_state_machine_commands_parser_cb)
    {
        s_state_machine_commands_parser_cb = _state_machine_commands_parser_cb;
    }
}