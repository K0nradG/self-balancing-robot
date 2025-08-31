#pragma once

class Drivers_Initializer
{
public:
    static void
    init();

private:
    static void
    reboot_on_error(int error);
};