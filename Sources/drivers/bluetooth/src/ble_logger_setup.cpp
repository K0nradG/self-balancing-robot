// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include "ble_connection.h"
#include "ble_setup.h"

bt_conn* my_conn = nullptr;

static bt_gatt_exchange_params exchange_params {};

static void
on_disconnected(bt_conn* conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(reason);
    set_con_status(false);
    set_att_payload_size(20u);
    bt_conn_unref(my_conn);
    my_conn = nullptr;
}

static void
exchange_func(bt_conn* conn, uint8_t att_err, bt_gatt_exchange_params* params)
{
    ARG_UNUSED(params);
    if(att_err == 0u)
    {
        uint16_t const payload_mtu = bt_gatt_get_mtu(conn) - 3u;  // 3 bytes used for Attribute headers.
        set_att_payload_size(payload_mtu);
    }
}

static void
update_data_length(bt_conn* conn)
{
    bt_conn_le_data_len_param my_data_len = {
        .tx_max_len  = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };

    bt_conn_le_data_len_update(conn, &my_data_len);
}

static void
update_mtu(bt_conn* conn)
{
    exchange_params.func = exchange_func;
    bt_gatt_exchange_mtu(conn, &exchange_params);
}

static void
update_phy(bt_conn* conn)
{
    bt_conn_le_phy_param const preferred_phy = {
        .options     = BT_CONN_LE_PHY_OPT_NONE,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };

    bt_conn_le_phy_update(conn, &preferred_phy);
}

static void
on_connected(bt_conn* conn, uint8_t err)
{
    if(err != 0)
    {
        return;
    }

    my_conn = bt_conn_ref(conn);
    set_att_payload_size(20u);

    update_phy(my_conn);
    update_data_length(my_conn);
    update_mtu(my_conn);

    set_con_status(true);
}

bt_conn_cb connection_callbacks = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

int
ble_init()
{
    bt_conn_cb_register(&connection_callbacks);
    return bt_enable(nullptr);
}