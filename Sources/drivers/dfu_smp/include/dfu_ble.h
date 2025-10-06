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


#ifdef __cplusplus
}
#endif
