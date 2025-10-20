#include <inttypes.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/logging/log.h>
#include "ble_connection.h"
#include "ble_setup.h"

LOG_MODULE_REGISTER(ble_setup, CONFIG_LOGGER_LOG_LEVEL);

struct bt_conn* my_conn = NULL;

static struct bt_gatt_exchange_params exchange_params = {0};

static const char*
phy2str(uint8_t phy)
{
    switch(phy)
    {
        case BT_GAP_LE_PHY_1M:
            return "1M";
        case BT_GAP_LE_PHY_2M:
            return "2M";
        case BT_GAP_LE_PHY_CODED:
            return "Coded";
        default:
            return "Unknown";
    }
}

static void
on_disconnected(struct bt_conn* conn, uint8_t reason)
{
    LOG_INF("Disconnected: reason %d", reason);
    set_con_status(false);
    bt_conn_unref(my_conn);
    my_conn = NULL;
}

static void
on_le_data_len_updated(struct bt_conn* conn, struct bt_conn_le_data_len_info* info)
{
    uint16_t tx_len  = info->tx_max_len;
    uint16_t tx_time = info->tx_max_time;
    uint16_t rx_len  = info->rx_max_len;
    uint16_t rx_time = info->rx_max_time;
    LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}

static void
exchange_func(struct bt_conn* conn, uint8_t att_err, struct bt_gatt_exchange_params* params)
{
    LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if(att_err == 0u)
    {
        uint16_t const payload_mtu = bt_gatt_get_mtu(conn) - 3u;
        LOG_INF("New MTU: %d bytes (payload)", payload_mtu);
    }
}

static void
update_data_length(struct bt_conn* conn)
{
    struct bt_conn_le_data_len_param len = {
        .tx_max_len  = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };

    int err = bt_conn_le_data_len_update(conn, &len);
    if(err)
    {
        LOG_ERR("bt_conn_le_data_len_update() failed (err %d)", err);
    }
}

static void
update_mtu(struct bt_conn* conn)
{
    exchange_params.func = exchange_func;
    int err              = bt_gatt_exchange_mtu(conn, &exchange_params);
    if(err)
    {
        LOG_ERR("bt_gatt_exchange_mtu() failed (err %d)", err);
    }
}

static void
update_phy(struct bt_conn* conn)
{
    struct bt_conn_le_phy_param phy_pref = {
        .options     = BT_CONN_LE_PHY_OPT_NONE,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
    };

    int err = bt_conn_le_phy_update(conn, &phy_pref);
    if(err)
    {
        LOG_ERR("bt_conn_le_phy_update() failed (err %d)", err);
    }
}

static void
on_le_param_updated(struct bt_conn* conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    LOG_INF("Connection parameters updated:");
    LOG_INF("  Interval: %.2f ms", interval * 1.25);
    LOG_INF("  Latency: %u", latency);
    LOG_INF("  Timeout: %u ms", timeout * 10);
}

static void
on_le_phy_updated(struct bt_conn* conn, struct bt_conn_le_phy_info* param)
{
    if(param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M)
    {
        LOG_INF("PHY updated. New PHY: 1M");
    }
    else if(param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M)
    {
        LOG_INF("PHY updated. New PHY: 2M");
    }
    else if(param->tx_phy == BT_CONN_LE_TX_POWER_PHY_CODED_S8)
    {
        LOG_INF("PHY updated. New PHY: Long Range");
    }
}

static void
on_connected(struct bt_conn* conn, uint8_t err)
{
    if(err)
    {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    my_conn = bt_conn_ref(conn);

    struct bt_conn_info info;
    if(bt_conn_get_info(conn, &info))
    {
        LOG_ERR("bt_conn_get_info() failed");
        return;
    }

    LOG_INF("Connected");
    LOG_INF("Connection parameters:");
    LOG_INF("  Interval: %.2f ms", info.le.interval * 1.25);
    LOG_INF("  Latency: %u", info.le.latency);
    LOG_INF("  Timeout: %u ms", info.le.timeout * 10);
    LOG_INF("  PHY TX/RX: %s/%s", phy2str(info.le.phy->tx_phy), phy2str(info.le.phy->rx_phy));
    LOG_INF("  MTU: %u bytes", bt_gatt_get_mtu(conn));

    k_sleep(K_MSEC(1000));
    update_phy(conn);
    update_data_length(conn);
    update_mtu(conn);
}

struct bt_conn_cb connection_callbacks = {
    .connected           = on_connected,
    .disconnected        = on_disconnected,
    .le_param_updated    = on_le_param_updated,
    .le_phy_updated      = on_le_phy_updated,
    .le_data_len_updated = on_le_data_len_updated,
};

int
ble_init(void)
{
    bt_conn_cb_register(&connection_callbacks);

    int ret = bt_enable(NULL);
    if(ret)
    {
        LOG_ERR("Bluetooth init failed (err %d)", ret);
        return ret;
    }

    LOG_INF("Bluetooth initialized");
    return 0;
}
