#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Registers custom shell commands.
 *
 * One of the available commands is `app_version`, which returns
 * the current firmware version running on the device:
 *
 *   uart:~$ app_version
 *   Application version: 0.3.0+0
 *
 * This allows checking which firmware version is currently active.
 *
 * Note:
 * To update the firmware, the version specified in the VERSION file
 * must be higher than the currently running version.
 */
void register_shell_commands();

#ifdef __cplusplus
}
#endif
