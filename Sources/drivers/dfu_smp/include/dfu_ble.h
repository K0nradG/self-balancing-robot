#ifndef DFU_BLE_H
#define DFU_BLE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*dfu_action_cb_t)(void);

void
dfu_action_cb_register(dfu_action_cb_t _dfu_action_cb);

void
get_app_version(void);

#ifdef __cplusplus
}
#endif

#endif  // DFU_BLE_H
