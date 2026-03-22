// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include <inttypes.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/logging/log.h>
#include "ble_connection.h"
#include "ble_setup.h"

LOG_MODULE_REGISTER(ble_setup, CONFIG_LOGGER_LOG_LEVEL);

bt_conn* my_conn = nullptr;

static bt_gatt_exchange_params exchange_params {};

static void
on_disconnected(bt_conn* conn, uint8_t reason)
{
    LOG_INF("Disconnected: %d", reason);
    set_con_status(false);
    bt_conn_unref(my_conn);
}

void
on_le_data_len_updated(bt_conn* conn, bt_conn_le_data_len_info* info)
{
    if(info != nullptr)
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
exchange_func(bt_conn* conn, uint8_t att_err, bt_gatt_exchange_params* params)
{
    LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if(att_err == 0u)
    {
        uint16_t const payload_mtu = bt_gatt_get_mtu(conn) - 3u;  // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}

static void
update_data_length(bt_conn* conn)
{
    bt_conn_le_data_len_param my_data_len = {
        .tx_max_len  = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };

    int const err = bt_conn_le_data_len_update(conn, &my_data_len);
    if(err != 0)
    {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

static void
update_mtu(bt_conn* conn)
{
    exchange_params.func = exchange_func;
    int const err        = bt_gatt_exchange_mtu(conn, &exchange_params);
    if(err != 0)
    {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}

static void
update_phy(bt_conn* conn)
{
    bt_conn_le_phy_param const preferred_phy = {
        .options     = BT_CONN_LE_PHY_OPT_NONE,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };

    int const err = bt_conn_le_phy_update(conn, &preferred_phy);
    if(err != 0)
    {
        LOG_ERR("bt_conn_le_phy_update() returned %d", err);
    }
}

static void
on_connected(bt_conn* conn, uint8_t err)
{
    if(err != 0)
    {
        LOG_INF("Connection failed: %d", err);
        return;
    }

    my_conn = bt_conn_ref(conn);
    bt_conn_info info {};

    err = bt_conn_get_info(conn, &info);
    if(err != 0)
    {
        LOG_ERR("bt_conn_get_info() returned %d", err);
        return;
    }

    double const connection_interval   = (double)info.le.interval * 1.25;  // in ms
    uint16_t const supervision_timeout = info.le.timeout * 10u;            // in ms
    LOG_INF(
        "Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval,
        info.le.latency, supervision_timeout);

    update_phy(my_conn);
    update_data_length(my_conn);
    update_mtu(my_conn);

    set_con_status(true);

    LOG_INF("Connected");
}

bt_conn_cb connection_callbacks = {
    .connected           = on_connected,
    .disconnected        = on_disconnected,
    .le_data_len_updated = on_le_data_len_updated,
};

int
ble_init()
{
    bt_conn_cb_register(&connection_callbacks);

    int const ret = bt_enable(nullptr);
    if(ret != 0)
    {
        LOG_ERR("Bluetooth init failed: %d", ret);
    }
    return ret;
}