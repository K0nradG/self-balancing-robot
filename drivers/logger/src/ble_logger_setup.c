#ifdef CONFIG_LOG_OVER_BLE

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include "ble_connection.h"
LOG_MODULE_REGISTER(ble_logger_setup, CONFIG_LOGGER_LOG_LEVEL);

#define DEVICE_NAME "SELF_BALANCING_ROBOT"
#define NO_SCAN_RSP_DATA 0

static struct bt_le_adv_param* adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),  // Connectable advertising and use identity address
    800,                                                       // Min Advertising Interval 500ms (800*0.625ms)
    801,                                                       // Max Advertising Interval 500.625ms (801*0.625ms)
    NULL);                                                     // Set to NULL for undirected advertising

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, strlen(DEVICE_NAME)),

};

static void
on_connected(struct bt_conn* conn, uint8_t err)
{
    if(err)
    {
        LOG_INF("Connection failed: %d", err);
        return;
    }
    set_con_status(true);
    LOG_INF("Connected");
}

static void
on_disconnected(struct bt_conn* conn, uint8_t reason)
{
    LOG_INF("Disconnected: %d", reason);
    set_con_status(false);
}

struct bt_conn_cb connection_callbacks = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

static void
bluetooth_init(int err)
{
    ARG_UNUSED(err);

    int ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, NO_SCAN_RSP_DATA);
    if(ret)
    {
        LOG_ERR("Advertising failed to start: %d", ret);
    }

    LOG_INF("Advertising successfully started");
}

static int
init(void)
{
    bt_conn_cb_register(&connection_callbacks);

    int ret = bt_enable(bluetooth_init);
    if(ret)
    {
        LOG_ERR("Bluetooth init failed: %d", ret);
    }
    return ret;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif