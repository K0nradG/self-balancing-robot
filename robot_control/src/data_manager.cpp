#include "data_manager.h"

DataManager::DataManager() : m_imu_data {}, m_encoders_data {} {}

DataManager&
DataManager::instance()
{
    static DataManager instance;
    return instance;
}

void
DataManager::update()
{
    m_imu_data = _get_imu_data();
    _get_encoders_data();
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

void
DataManager::_set_encoders_data(encoders_data data)
{
    m_encoders_data = data;
}

void
new_encoder_data_callback(encoders_data encoders_data)
{
    DataManager::instance()._set_encoders_data(encoders_data);
}

static int
init(void)
{
    new_encoder_data_updated_cb_register(new_encoder_data_callback);
    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
