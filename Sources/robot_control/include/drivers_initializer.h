// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

namespace Robot_Control
{

class Drivers_Initializer
{
public:
    static void
    init();

private:
    static void
    reboot_on_error(int error);
};

}  // namespace Robot_Control