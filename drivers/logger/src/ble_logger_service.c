#ifdef CONFIG_LOG_OVER_BLE

#include "ble_logger_service.h"
#include <bluetooth/services/nus.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include "ble_connection.h"
LOG_MODULE_REGISTER(ble_logger_nus, CONFIG_LOGGER_LOG_LEVEL);

static bool nus_notification_enabled;

static void
nus_data_received(struct bt_conn* conn, const uint8_t* data, uint16_t len)
{
    LOG_INF("NUS received data: %.*s", len, data);
}

void
nus_notif_enabled(enum bt_nus_send_status status)
{
    if(status == BT_NUS_SEND_STATUS_ENABLED)
    {
        nus_notification_enabled = true;
        LOG_INF("notification enabled");
    }
    else if(status == BT_NUS_SEND_STATUS_DISABLED)
    {
        nus_notification_enabled = false;
        LOG_INF("notification disabled");
    }
}

static struct bt_nus_cb nus_callbacks = {
    .send_enabled = nus_notif_enabled,
    .received     = nus_data_received,
};

static int
init()
{
    int err = bt_nus_init(&nus_callbacks);
    if(err)
    {
        LOG_ERR("NUS initialization error: %d", err);
    }
    return err;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

void
ble_logger_send(char* data)
{
    if(get_con_status() && nus_notification_enabled)
    {
        int err = bt_nus_send(NULL, data, strlen(data));
        if(err)
        {
            LOG_ERR("NUS failed to send data: %d", err);
        }
    }
}

#endif /*CONFIG_LOG_OVER_BLE*/