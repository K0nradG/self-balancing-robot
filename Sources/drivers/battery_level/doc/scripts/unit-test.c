// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include <stdint.h>
#include <stdio.h>

static uint8_t
battery_charge_level(int16_t voltage_mv)
{
    static uint8_t previous_charge_level = 100;
    uint8_t charge_level;
    int16_t slope;
    int32_t intercept;

    if(voltage_mv > 8400)  // 100%
    {
        charge_level = 100;
    }
    else if(voltage_mv > 7900)  // 100% - 80%
    {
        slope        = (100 - 80) / (8400 - 7900);
        intercept    = 100 - slope * 8400;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 7400)  // 80% - 60%
    {
        slope        = (80 - 60) / (7900 - 7400);
        intercept    = 80 - slope * 7900;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 7000)  // 60% - 30%
    {
        slope        = (60 - 30) / (7400 - 7000);
        intercept    = 60 - slope * 7400;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 6600)  // 30% - 10%
    {
        slope        = (30 - 10) / (7000 - 6600);
        intercept    = 30 - slope * 7000;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 6000)  // 10% - 0%
    {
        slope        = (10 - 0) / (6600 - 6000);
        intercept    = 10 - slope * 6600;
        charge_level = slope * voltage_mv + intercept;
    }
    else  // 0%
    {
        charge_level = 0;
    }

    if(charge_level > 100)
    {
        charge_level = 100;
    }
    else if(charge_level < 0)
    {
        charge_level = 0;
    }

    if(previous_charge_level < charge_level)
    {
        charge_level = previous_charge_level;
    }
    previous_charge_level = charge_level;

    return charge_level;
}

int
main()
{
    // Tablica przykładowych napięć baterii (mV)
    int16_t test_voltages[] = {8500, 8200, 7900, 7700, 7400, 7200, 7000, 6800, 6600, 6400, 6000, 5800};
    int size                = sizeof(test_voltages) / sizeof(test_voltages[0]);

    printf("Voltage (mV) | Charge Level (%%)\n");
    printf("-------------------------------\n");

    for(int i = 0; i < size; i++)
    {
        int16_t voltage = test_voltages[i];
        uint8_t charge  = battery_charge_level(voltage);
        printf("%6d mV    | %3d%%\n", voltage, charge);
    }

    return 0;
}
