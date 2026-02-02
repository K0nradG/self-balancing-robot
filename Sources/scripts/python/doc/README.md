# Python scripts overview

## Clangd issues fix

In order for `clangd` to work properly, `compile_commands.json` must be acquired from `CMake` build. The `get_compile_commands.py` takes the generated file from build directory and formats it by getting rid of flags not recognized by `clang`.

The `build_dongle_sw.py` script already obtains the appropriate file, however whenever compilation warnings/errors occur, one can run the `get_compile_commands.py` script and reload the VS Code window (ctrl + shift + p). This should fix the issues.

## Formatting files

For this task we are using the `clang-format` tool.

The `format_all_files.py` script can be used to manually format all source and header files present in the repo.

The `format_changed_files.py` can also be used, however it is automatically launched during FW build with the use of `build_dongle_sw.py` script.

## FW flashing with external debugger

The `flash_dongle.py` wraps the commands necessary to flash the *nRF52840* board with a pre-built FW. This has to be done with the use of an **external debugger**, e.g. the *nRF7002dk* board. 

## Building nRF7002dk firmware

The `build_dk_sw.py` script allows for building the *nRF7002dk* FW from `Sources/applications/controller`.