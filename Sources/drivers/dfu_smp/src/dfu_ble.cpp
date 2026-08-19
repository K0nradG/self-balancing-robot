// Copyright 2026 Filip Dymczyk and Konrad Grucel

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
#include "ble_protocol.h"
#include "ble_service.h"
#include "control_loop.h"

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
static uint32_t g_dfu_request_sequence;

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

static void
send_dfu_command_result(uint32_t request_sequence, BLE_Protocol::Command_Status status)
{
    uint8_t result[6] {};
    BLE_Protocol::put_u32(result, request_sequence);
    result[4] = static_cast<uint8_t>(BLE_Protocol::Message_Type::DFU_COMMAND);
    result[5] = static_cast<uint8_t>(status);
    ble_send_packet(BLE_Protocol::Message_Type::COMMAND_RESULT, result, sizeof(result));
}

void
dfu_packet_received(BLE_Protocol::Packet_View const& packet)
{
    BLE_Protocol::Command_Status status = BLE_Protocol::Command_Status::OK;
    if(packet.payload_length != 1u)
    {
        status = BLE_Protocol::Command_Status::INVALID_LENGTH;
    }
    else
    {
        switch(static_cast<BLE_Protocol::Dfu_Action>(packet.payload[0]))
        {
            case BLE_Protocol::Dfu_Action::START:
                LOG_INF("DFU START command received");
                g_dfu_state = DFU_STATE_START;
                break;

            case BLE_Protocol::Dfu_Action::SKIP:
                LOG_INF("DFU SKIP command received");
                g_dfu_state            = DFU_STATE_SKIP;
                g_dfu_request_sequence = packet.sequence;
                break;

            default:
                status = BLE_Protocol::Command_Status::INVALID_VALUE;
                break;
        }
    }

    if(status == BLE_Protocol::Command_Status::OK)
    {
        if(g_dfu_state == DFU_STATE_START)
        {
            send_dfu_command_result(packet.sequence, status);
        }
        k_sem_give(&dfu_sem);
    }
    else
    {
        send_dfu_command_result(packet.sequence, status);
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
        send_dfu_command_result(g_dfu_request_sequence, BLE_Protocol::Command_Status::OK);
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

#if defined(CONFIG_BATTERY_LEVEL_DRV) && !defined(CONFIG_MODEL_IDENTIFICATION_DRV)

#define MEASUREMENT_INTERVAL 9000

static void
new_battery_level_callback(battery_level_data data)
{
    uint8_t payload[4] {};
    BLE_Protocol::put_u16(payload, data.battery_level_mv);
    payload[2] = data.battery_level_percent;
    ble_send_packet(BLE_Protocol::Message_Type::BATTERY_STATUS, payload, sizeof(payload));
}
#endif  // CONFIG_BATTERY_LEVEL_DRV

static int
dfu_smp_init()
{
    int ret;

#if defined(CONFIG_BATTERY_LEVEL_DRV) && !defined(CONFIG_MODEL_IDENTIFICATION_DRV)
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

    ble_dfu_packet_received_cb_register(dfu_packet_received);
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