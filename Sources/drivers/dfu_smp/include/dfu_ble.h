#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*dfu_action_cb_t)(void);

void
dfu_action_cb_register(dfu_action_cb_t _dfu_action_cb);

#ifdef __cplusplus
}
#endif
