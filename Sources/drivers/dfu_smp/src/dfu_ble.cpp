#include <inttypes.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include "ble_commands.h"
#include "ble_service.h"
#include "control_loop.h"
#include "logger.h"

/*TODO: now dfu is mandatory so BLE needs to be default y*/
#include "ble_service.h"
#include "ble_setup.h"
#include "dfu_ble.h"

#ifdef CONFIG_INTERFACE_DRV
#include "interface.h"
#endif

#ifdef CONFIG_SHELL_DRV
#include "shell.h"
#endif

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"
#endif  // CONFIG_BATTERY_LEVEL_DRV

LOG_MODULE_REGISTER(dfu_ble, CONFIG_DFU_BLE_LOG_LEVEL);

#define DFU_BLINKING_INTERVAL 100

static bt_le_adv_param const* adv_param =
    BT_LE_ADV_PARAM((BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY), 800, 801, nullptr);

static const bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, SMP_BT_SVC_UUID_VAL),
};

static const bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

typedef enum
{
    DFU_STATE_WAITING,
    DFU_STATE_SKIP,
    DFU_STATE_START,
} dfu_state_t;

static dfu_state_t g_dfu_state = DFU_STATE_WAITING;

static dfu_action_cb_t dfu_action_cb;

K_SEM_DEFINE(dfu_sem, 0, 1);

static void
start_smp_adv_handler(k_work* work)
{
    int ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if(ret != 0)
    {
        LOG_ERR("Advertising failed to start: %d", ret);
        return;
    }
    LOG_INF("SMP advertising started - ready for DFU");
}

K_WORK_DELAYABLE_DEFINE(dfu_smp_start_adv_work, start_smp_adv_handler);

static void
start_dfu_smp_adv()
{
    k_work_submit(&dfu_smp_start_adv_work.work);
}

static enum mgmt_cb_return
upload_confirm_handler(uint32_t, enum mgmt_cb_return, int32_t* rc, uint16_t*, bool*, void* data, size_t)
{
    const img_mgmt_upload_check* imgData = (const img_mgmt_upload_check*)data;
    LOG_INF(
        "DFU over SMP progress: %" PRIu64 " / %" PRIu64 " B (image: %u)", (uint64_t)imgData->req->off,
        (uint64_t)imgData->action->size, imgData->req->image);
    return MGMT_CB_OK;
}

static mgmt_callback sUploadCallback = {
    .callback = upload_confirm_handler,
    .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK,
};

void
dfu_process_parser_cb(const char* payload)
{
    if(!payload || *payload == '\0')
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
        case BLE_Commands::DFU::DFU_START:
            LOG_INF("DFU START command received");
            g_dfu_state = DFU_STATE_START;
            k_sem_give(&dfu_sem);
            break;

        case BLE_Commands::DFU::DFU_SKIP:
            LOG_INF("DFU SKIP command received");
            g_dfu_state = DFU_STATE_SKIP;
            k_sem_give(&dfu_sem);
            break;

        default:
            LOG_WRN("DFU unknown command: %c", key);
            break;
    }
}

K_THREAD_STACK_DEFINE(dfu_wait_stack, 1024);
static k_thread dfu_wait_thread_data;

static void
dfu_wait_thread(void* arg1, void* arg2, void* arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    LOG_INF("Waiting for DFU command...");

    k_sem_take(&dfu_sem, K_FOREVER);

    if(g_dfu_state == DFU_STATE_SKIP)
    {
        LOG_INF("DFU skipped, starting main application...");
        Robot_Control::control_loop_init();
        if(dfu_action_cb)
        {
            dfu_action_cb();
        }
        return;
    }

    if(g_dfu_state == DFU_STATE_START)
    {
        LOG_INF("Entering DFU mode...");
        get_app_version();

        // Keep thread alive but not blocking system
        while(1)
        {
            k_msleep(1000);
        }
    }
}

static void
confirm_new_image()
{
    int err = mcuboot_swap_type();
    if(err != BOOT_SWAP_TYPE_REVERT)
        return;

    if(boot_write_img_confirmed())
    {
        LOG_ERR("Failed to confirm firmware image - will revert on next boot");
    }
    else
    {
        LOG_INF("New firmware image confirmed");
    }
}

#ifdef CONFIG_BATTERY_LEVEL_DRV

#define MEASUREMENT_INTERVAL 9000

static Logger<IS_ENABLED(1)> boot_state_logger("BOOT");

static void
new_battery_level_callback(battery_level_data data)
{
    boot_state_logger.platform_log(LOG_LEVEL::INF, "bat lvl %u", data.battery_level_percent);
    boot_state_logger.platform_log(LOG_LEVEL::INF, "bat lvl mv %u", data.battery_level_mv);
}
#endif  // CONFIG_BATTERY_LEVEL_DRV

static int
dfu_smp_init()
{
    int ret;

#ifdef CONFIG_BATTERY_LEVEL_DRV
    new_battery_level_cb_register(new_battery_level_callback);
    battery_start_periodic_measurement(MEASUREMENT_INTERVAL);
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_BATTERY_LEVEL_DRV
    ret = battery_level_init();
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_INTERFACE_DRV
    ret = interface_init();
    led_start_periodic_blinking(DFU_BLINKING_INTERVAL);
#endif

    ret = ble_init();
    ret = ble_service_init();

    confirm_new_image();

    dfu_process_parser_cb_register(dfu_process_parser_cb);
    mgmt_callback_register(&sUploadCallback);

    start_dfu_smp_adv();
    k_thread_create(
        &dfu_wait_thread_data, dfu_wait_stack, K_THREAD_STACK_SIZEOF(dfu_wait_stack), dfu_wait_thread, nullptr, nullptr,
        nullptr, 7, 0, K_NO_WAIT);

    LOG_INF("DFU SMP initialization done");
    return 0;
}

SYS_INIT(dfu_smp_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

void
dfu_action_cb_register(dfu_action_cb_t _dfu_action_cb)
{
    if(_dfu_action_cb)
    {
        dfu_action_cb = _dfu_action_cb;
    }
}