// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include "ble_service.h"
#include "dfu_ble.h"
#include "logger.h"

static Logger<true> app_version_logger("APP_VERSION");

static void
get_app_version_work_handler(k_work* work)
{
    ARG_UNUSED(work);
    mcuboot_img_header hdr;
    int rc = boot_read_bank_header(FLASH_AREA_ID(image_0), &hdr, sizeof(hdr));

    if(rc != 0)
    {
        app_version_logger.platform_log(LOG_LEVEL::INF, "Failed to read image header (rc=%d)", rc);
        return;
    }

    uint8_t payload[8] {};
    BLE_Protocol::Payload_Writer writer(payload, sizeof(payload));
    writer.put_u8(hdr.h.v1.sem_ver.major);
    writer.put_u8(hdr.h.v1.sem_ver.minor);
    writer.put_u8(static_cast<uint8_t>(hdr.h.v1.sem_ver.revision));
    writer.put_u8(0u);
    writer.put_u32(hdr.h.v1.sem_ver.build_num);
    ble_send_packet(BLE_Protocol::Message_Type::APP_VERSION, writer);
}

K_WORK_DELAYABLE_DEFINE(get_app_version_work, get_app_version_work_handler);

void
get_app_version()
{
    k_work_submit(&get_app_version_work.work);
}