#pragma once

typedef void (*dfu_action_cb_t)();

void
dfu_action_cb_register(dfu_action_cb_t _dfu_action_cb);

void
get_app_version();
