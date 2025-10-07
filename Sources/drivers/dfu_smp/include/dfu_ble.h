#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int
dfu_smp_init();

void
confirm_new_image();

void
start_dfu_smp_adv();

void
check_for_software_updates();

bool
get_software_update_status();

#ifdef __cplusplus
}
#endif
