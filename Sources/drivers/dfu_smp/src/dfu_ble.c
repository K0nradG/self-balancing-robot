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
#include "ble_commands.h"
#include "ble_service.h"

LOG_MODULE_REGISTER(dfu_ble, CONFIG_DFU_BLE_LOG_LEVEL);

#define DEVICE_NAME "SELF_BALANCING_ROBOT"

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

typedef enum
{
    DFU_STATE_WAITING,
    DFU_STATE_SKIP,
    DFU_STATE_START
} dfu_state_t;

static dfu_state_t g_dfu_state = DFU_STATE_WAITING;

K_SEM_DEFINE(dfu_sem, 0, 1);

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

static enum mgmt_cb_return
UploadConfirmHandler(uint32_t, enum mgmt_cb_return, int32_t* rc, uint16_t*, bool*, void* data, size_t)
{
    const struct img_mgmt_upload_check* imgData = (const struct img_mgmt_upload_check*)data;

    LOG_INF(
        "DFU over SMP progress: %" PRIu64 " / %" PRIu64 " B of image: %u", (uint64_t)imgData->req->off,
        (uint64_t)imgData->action->size, imgData->req->image);

    return MGMT_CB_OK;
}

static struct mgmt_callback sUploadCallback = {
    .callback = UploadConfirmHandler,
    .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK,
};

void
start_dfu_smp_adv()
{
    k_work_submit(&dfu_smp_start_adv_work.work);
}

void
confirm_new_image()
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

void
dfu_process_parser_cb(const char* payload)
{
    if(payload == NULL || *payload == '\0')
    {
        LOG_ERR("DFU parser: empty payload");
        return;
    }

    // skip DFU_PREFIX
    payload++;

    if(*payload == '\0')
    {
        LOG_ERR("DFU parser: missing command after prefix");
        return;
    }

    char key = payload[0];

    switch(key)
    {
        case DFU_START_CMD:
            LOG_INF("DFU START command received");
            g_dfu_state = DFU_STATE_START;
            k_sem_give(&dfu_sem);
            break;

        case DFU_SKIP_CMD:
            LOG_INF("DFU SKIP command received");
            g_dfu_state = DFU_STATE_SKIP;
            k_sem_give(&dfu_sem);
            break;

        default:
            LOG_WRN("DFU unknown command: %c", key);
            break;
    }
}

void
check_for_software_updates()
{
    LOG_INF("Waiting for DFU command...");

    k_sem_take(&dfu_sem, K_FOREVER);

    if(g_dfu_state == DFU_STATE_SKIP)
    {
        LOG_INF("DFU skipped, continuing initialization...");
        return;
    }

    if(g_dfu_state == DFU_STATE_START)
    {
        LOG_INF("Entering DFU mode...");
        while(1)
        {
            k_msleep(1000);  // waiting here for SMP client DFU initialization
        }
    }
}

int
dfu_smp_init()
{
    dfu_process_parser_cb_register(dfu_process_parser_cb);
    mgmt_callback_register(&sUploadCallback);
    return 0;
}
