# Memory report

To get the **memory report** execute:


    cd build
    ninja partition_manager_report

Result:

| Region                 | Start Address | Size (Hex) | Size    | Description             |
| ---------------------- | ------------- | ---------- | ------- | ----------------------- |
| Flash                  | 0x00000000    | 0x100000   | 1024 kB | Total flash memory      |
| MCUboot                | 0x00000000    | 0x8000     | 32 kB   | MCUboot bootloader      |
| MCUboot Primary Slot   | 0x00008000    | 0x7C000    | 496 kB  | Primary firmware slot   |
| MCUboot Pad            | 0x00008000    | 0x200      | 512 B   | Image padding           |
| Primary App            | 0x00008200    | 0x7BE00    | 495 kB  | Application image       |
| MCUboot Secondary Slot | 0x00084000    | 0x7C000    | 496 kB  | Secondary firmware slot |
