// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include "dfu_ble.h"
#include "logger.h"

static Logger<true> app_version_logger("APP_VERSION");

static void
get_app_version_work_handler(k_work* work)
{
    mcuboot_img_header hdr;
    int rc = boot_read_bank_header(FLASH_AREA_ID(image_0), &hdr, sizeof(hdr));

    if(rc != 0)
    {
        app_version_logger.platform_log(LOG_LEVEL::INF, "Failed to read image header (rc=%d)", rc);
        return;
    }

    app_version_logger.platform_log(
        LOG_LEVEL::INF, "App version: %u.%u.%u+%u", hdr.h.v1.sem_ver.major, hdr.h.v1.sem_ver.minor,
        hdr.h.v1.sem_ver.revision, hdr.h.v1.sem_ver.build_num);
}

K_WORK_DELAYABLE_DEFINE(get_app_version_work, get_app_version_work_handler);

void
get_app_version()
{
    k_work_submit(&get_app_version_work.work);
}