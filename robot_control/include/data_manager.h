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

    /*it is hidden API used only by enocder data update callback
    TODO: get rid of this callabck*/
    void
    _set_encoders_data(encoders_data data);

private:
    DataManager();
    DataManager(const DataManager&) = delete;
    DataManager&
    operator=(const DataManager&) = delete;

    imu_data m_imu_data;
    encoders_data m_encoders_data;
};
