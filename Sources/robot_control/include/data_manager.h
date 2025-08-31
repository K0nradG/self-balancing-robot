#pragma once

#include "encoder.h"
#include "imu.h"

class DataManager
{
public:
    static DataManager&
    instance();

    void
    update();

    imu_data
    get_imu_data() const;

    encoders_data
    get_encoders_data() const;

private:
    DataManager() = default;

    DataManager(DataManager const&) = delete;

    DataManager&
    operator=(DataManager const&) = delete;

    imu_data m_imu_data {};
    encoders_data m_encoders_data {};
};
