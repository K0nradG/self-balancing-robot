#include <inttypes.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include "ble_connection.h"
#include "ble_setup.h"

LOG_MODULE_REGISTER(ble_setup, CONFIG_LOGGER_LOG_LEVEL);

#define DEVICE_NAME "SELF_BALANCING_ROBOT"

struct bt_conn* my_conn                               = NULL;
static struct bt_gatt_exchange_params exchange_params = {0};

static struct bt_le_adv_param const* adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY),  // Connectable advertising and use identity address
    800,                                                       // Min Advertising Interval 500ms (800*0.625ms)
    801,                                                       // Max Advertising Interval 500.625ms (801*0.625ms)
    NULL);                                                     // Set to NULL for undirected advertising

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, SMP_BT_SVC_UUID_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, sizeof(DEVICE_NAME) - 1),
};

static void
start_smp_adv_handler(struct k_work* work)
{
    int const ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if(ret != 0)
    {
        LOG_ERR("Advertising failed to start: %d", ret);
        return;
    }

    LOG_INF("Advertising successfully started");
}

K_WORK_DELAYABLE_DEFINE(dfu_smp_start_adv_work, start_smp_adv_handler);

static void
on_disconnected(struct bt_conn* conn, uint8_t reason)
{
    LOG_INF("Disconnected: %d", reason);
    set_con_status(false);
    bt_conn_unref(my_conn);
    // k_work_schedule(&dfu_smp_start_adv_work, K_MSEC(100));
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
    if(att_err == 0u)
    {
        uint16_t const payload_mtu = bt_gatt_get_mtu(conn) - 3u;  // 3 bytes used for Attribute headers.
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
    if(err != 0)
    {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

static void
update_mtu(struct bt_conn* conn)
{
    exchange_params.func = exchange_func;
    int const err        = bt_gatt_exchange_mtu(conn, &exchange_params);
    if(err != 0)
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
    if(err != 0)
    {
        LOG_ERR("bt_conn_le_phy_update() returned %d", err);
    }
}

static void
on_connected(struct bt_conn* conn, uint8_t err)
{
    if(err != 0)
    {
        LOG_INF("Connection failed: %d", err);
        return;
    }

    my_conn                  = bt_conn_ref(conn);
    struct bt_conn_info info = {0};

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

    // update_phy(my_conn);

    // update_data_length(my_conn);
    // update_mtu(my_conn);

    set_con_status(true);

    LOG_INF("Connected");
}

struct bt_conn_cb connection_callbacks = {
    .connected           = on_connected,
    .disconnected        = on_disconnected,
    .le_data_len_updated = on_le_data_len_updated,
};

enum mgmt_cb_return
UploadConfirmHandler(uint32_t, enum mgmt_cb_return, int32_t* rc, uint16_t*, bool*, void* data, size_t)
{
    const struct img_mgmt_upload_check* imgData = (const struct img_mgmt_upload_check*)data;

    LOG_INF(
        "DFU over SMP progress: %zu / %zu B of image: %zu", imgData->req->off, imgData->action->size,
        imgData->req->image);

    return MGMT_CB_OK;
}

enum mgmt_cb_return
DfuStoppedHandler(uint32_t, enum mgmt_cb_return, int32_t*, uint16_t*, bool*, void*, size_t)
{
    return MGMT_CB_OK;
}

struct mgmt_callback sUploadCallback = {
    .callback = UploadConfirmHandler,
    .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK,
};

struct mgmt_callback sDfuStopped = {
    .callback = DfuStoppedHandler,
    .event_id = (MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED | MGMT_EVT_OP_IMG_MGMT_DFU_PENDING),
};

int
ble_init(void)
{
    mgmt_callback_register(&sUploadCallback);
    mgmt_callback_register(&sDfuStopped);

    bt_conn_cb_register(&connection_callbacks);

    int const ret = bt_enable(NULL);
    if(ret != 0)
    {
        LOG_ERR("Bluetooth init failed: %d", ret);
    }
    k_work_submit(&dfu_smp_start_adv_work.work);

    return ret;
}

void
ConfirmNewImage()
{
    /* Check if the image is run in the REVERT mode and eventually */
    /* confirm it to prevent reverting on the next boot. */
    int err = mcuboot_swap_type();
    if(err != BOOT_SWAP_TYPE_REVERT)
    {
        return;
    }

    if(boot_write_img_confirmed())
    {
        LOG_ERR("Failed to confirm firmware image, it will be reverted on the next boot");
    }
    else
    {
        LOG_INF("New firmware image confirmed");
    }
}