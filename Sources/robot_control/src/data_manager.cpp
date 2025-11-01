#include "data_manager.h"

namespace Robot_Control
{

DataManager&
DataManager::instance()
{
    static DataManager instance;
    return instance;
}

void
DataManager::update()
{
    m_imu_data       = _get_imu_data();
    m_encoders_data  = _get_encoders_data();
    m_rotation_angle = alpha_rotation * (m_rotation_angle + m_imu_data.angle_rotation_dt * m_imu_data.time_dt) +
                       (1.0f - alpha_rotation) * m_encoders_data.robot_angle_rad;
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

float
DataManager::get_rotation_angle() const
{
    return m_rotation_angle;
}

void
DataManager::reset()
{
    reset_imu_balance_angle();
    reset_encoders();
    m_rotation_angle = 0.0f;
}

}  // namespace Robot_Control