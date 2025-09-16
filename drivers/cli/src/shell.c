#include "shell.h"
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/flash_map.h>

static int
cmd_app_version(const struct shell* sh, size_t argc, char** argv)
{
    struct mcuboot_img_header hdr;
    int rc = boot_read_bank_header(FLASH_AREA_ID(image_0), &hdr, sizeof(hdr));

    if(rc != 0)
    {
        shell_error(sh, "Failed to read image header (rc=%d)", rc);
        return rc;
    }

    shell_print(
        sh, "Application version: %u.%u.%u+%u", hdr.h.v1.sem_ver.major, hdr.h.v1.sem_ver.minor,
        hdr.h.v1.sem_ver.revision, hdr.h.v1.sem_ver.build_num);
    return 0;
}

void
register_shell_commands(void)
{
    SHELL_CMD_REGISTER(app_version, NULL, "Show application version from image header", cmd_app_version);
}
