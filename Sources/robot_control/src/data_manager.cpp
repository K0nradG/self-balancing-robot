#include "data_manager.h"

DataManager&
DataManager::instance()
{
    static DataManager instance;
    return instance;
}

void
DataManager::update()
{
    m_imu_data      = _get_imu_data();
    m_encoders_data = _get_encoders_data();
}

imu_data
DataManager::get_imu_data() const
{
    return m_imu_data;
}

encoders_data
DataManager::get_encoders_data() const
{
    return m_encoders_data;
}
