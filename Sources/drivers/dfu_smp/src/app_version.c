#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include "dfu_ble.h"
#include "logger.h"

static void
get_app_version_work_handler(struct k_work* work)
{
    struct mcuboot_img_header hdr;
    int rc = boot_read_bank_header(FLASH_AREA_ID(image_0), &hdr, sizeof(hdr));

    if(rc != 0)
    {
        platform_log("APP_VERSION", LOG_LEVEL_ERR, "Failed to read image header (rc=%d)", rc);
        return;
    }

    platform_log(
        "APP_VERSION", LOG_LEVEL_INF, "App version: %u.%u.%u+%u", hdr.h.v1.sem_ver.major, hdr.h.v1.sem_ver.minor,
        hdr.h.v1.sem_ver.revision, hdr.h.v1.sem_ver.build_num);
}

K_WORK_DELAYABLE_DEFINE(get_app_version_work, get_app_version_work_handler);

void
get_app_version(void)
{
    k_work_submit(&get_app_version_work.work);
}
