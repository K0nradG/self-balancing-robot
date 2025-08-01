#ifdef CONFIG_LOG_OVER_BLE

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include "ble_connection.h"

LOG_MODULE_REGISTER(ble_logger_setup, CONFIG_LOGGER_LOG_LEVEL);

#define DEVICE_NAME      "SELF_BALANCING_ROBOT"
#define NO_SCAN_RSP_DATA 0

struct bt_conn* my_conn                               = NULL;
static struct bt_gatt_exchange_params exchange_params = {0};

static struct bt_le_adv_param const* adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),  // Connectable advertising and use identity address
    800,                                                       // Min Advertising Interval 500ms (800*0.625ms)
    801,                                                       // Max Advertising Interval 500.625ms (801*0.625ms)
    NULL);                                                     // Set to NULL for undirected advertising
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, strlen(DEVICE_NAME)),

};

static void
on_disconnected(struct bt_conn* conn, uint8_t reason)
{
    LOG_INF("Disconnected: %d", reason);
    set_con_status(false);
    bt_conn_unref(my_conn);
}

void
on_le_data_len_updated(struct bt_conn* conn, struct bt_conn_le_data_len_info* info)
{
    if(info != NULL)
    {
        uint16_t tx_len  = info->tx_max_len;
        uint16_t tx_time = info->tx_max_time;
        uint16_t rx_len  = info->rx_max_len;
        uint16_t rx_time = info->rx_max_time;
        LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
    }
    else
    {
        LOG_ERR("Wrong info on le data length update!");
    }
}

static void
exchange_func(struct bt_conn* conn, uint8_t att_err, struct bt_gatt_exchange_params* params)
{
    LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if(!att_err)
    {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;  // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}

static void
update_data_length(struct bt_conn* conn)
{
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len  = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };

    int const err = bt_conn_le_data_len_update(conn, &my_data_len);
    if(err)
    {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

static void
update_mtu(struct bt_conn* conn)
{
    exchange_params.func = exchange_func;
    int const err        = bt_gatt_exchange_mtu(conn, &exchange_params);
    if(err)
    {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}

static void
update_phy(struct bt_conn* conn)
{
    struct bt_conn_le_phy_param const preferred_phy = {
        .options     = BT_CONN_LE_PHY_OPT_NONE,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
    };

    int const err = bt_conn_le_phy_update(conn, &preferred_phy);
    if(err)
    {
        LOG_ERR("bt_conn_le_phy_update() returned %d", err);
    }
}

static void
on_connected(struct bt_conn* conn, uint8_t err)
{
    if(err)
    {
        LOG_INF("Connection failed: %d", err);
        return;
    }

    my_conn                  = bt_conn_ref(conn);
    struct bt_conn_info info = {0};

    err = bt_conn_get_info(conn, &info);
    if(err)
    {
        LOG_ERR("bt_conn_get_info() returned %d", err);
        return;
    }

    double connection_interval   = info.le.interval * 1.25;  // in ms
    uint16_t supervision_timeout = info.le.timeout * 10;     // in ms
    LOG_INF(
        "Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval,
        info.le.latency, supervision_timeout);

    update_phy(my_conn);

    update_data_length(my_conn);
    update_mtu(my_conn);

    set_con_status(true);

    LOG_INF("Connected");
}

struct bt_conn_cb connection_callbacks = {
    .connected           = on_connected,
    .disconnected        = on_disconnected,
    .le_data_len_updated = on_le_data_len_updated,
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