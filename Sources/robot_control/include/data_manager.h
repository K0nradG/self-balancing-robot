#pragma once

#include "encoder.h"
#include "imu.h"

namespace Robot_Control
{

class DataManager
{
    static constexpr float alpha_rotation = 0.8f;

public:
    static DataManager&
    instance();

    void
    update();

    imu_data
    get_imu_data() const;

    encoders_data const&
    get_encoders_data() const;

    float
    get_rotation_angle() const;

    void
    reset();

private:
    DataManager() = default;

    DataManager(DataManager const&) = delete;

    DataManager&
    operator=(DataManager const&) = delete;

    imu_data m_imu_data {};
    encoders_data m_encoders_data {};
    float m_rotation_angle {};
};

}  // namespace Robot_Control