// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_handler.h"
#include "ble_payload_writer.h"
#include "ble_transfer_handler.h"
#include "control_loop.h"
#include "main_state_machine.h"
#include "robot_controller.h"

namespace Robot_Control
{

void
BLE_Handler::handle_received_packet(BLE_Protocol::Received_Packet const& received_packet)
{
    if(m_previous_packet_handling_in_progress)
    {
        send_command_result(received_packet, BLE_Protocol::Command_Status::INVALID_STATE);
        return;
    }

    m_previous_packet_handling_in_progress = true;
    auto status                            = BLE_Protocol::Command_Status::OK;

    BLE_Protocol::Payload_Reader reader(received_packet.payload, received_packet.payload_length);
    switch(received_packet.type)
    {
        case BLE_Protocol::Message_Type::STATE_COMMAND:
        {
            status = Main_State_Machine::instance().receive_command(reader);
            break;
        }
        case BLE_Protocol::Message_Type::GET_PID_STATE:
        {
            if(!reader.done())
            {
                status = BLE_Protocol::Command_Status::INVALID_LENGTH;
                break;
            }

            Control_Loop::instance().send_PID_controllers_parameters();
            break;
        }
        case BLE_Protocol::Message_Type::SET_PID:
        {
            status = Control_Loop::instance().set_PID_parameters(reader);
            break;
        }
        case BLE_Protocol::Message_Type::SET_SETPOINT:
        {
            status = Robot_Controller::instance().set_setpoints(reader);
            break;
        }
        case BLE_Protocol::Message_Type::TRAJECTORY_COMMAND:
        {
            status = Robot_Controller::instance().set_trajectory_command(reader);
            break;
        }
        case BLE_Protocol::Message_Type::SET_LQR:
        {
#ifndef CONFIG_PID_ENABLED
            status = Control_Loop::instance().set_LQR_parameters(reader);
#else
            status = BLE_Protocol::Command_Status::UNSUPPORTED_MESSAGE;
#endif
            break;
        }
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
        case BLE_Protocol::Message_Type::IDENTIFICATION_CONFIG:
        {
            if(received_packet.payload_length != (10u * 2u * sizeof(float)))
            {
                status = BLE_Protocol::Command_Status::INVALID_LENGTH;
                break;
            }

            status = Model_Identification::instance().set_identification_profile(
                         received_packet.payload, received_packet.payload_length) ?
                         BLE_Protocol::Command_Status::OK :
                         BLE_Protocol::Command_Status::INVALID_VALUE;
            break;
        }
#endif
        default:
        {
            status = BLE_Protocol::Command_Status::UNSUPPORTED_MESSAGE;
            break;
        }
    }

    send_command_result(received_packet, status);

    m_previous_packet_handling_in_progress = false;
}

void
BLE_Handler::send_command_result(
    BLE_Protocol::Received_Packet const& received_packet, BLE_Protocol::Command_Status status)
{
    uint8_t payload[6] {};
    BLE_Protocol::Payload_Writer writer(payload, sizeof(payload));
    writer.put_u32(received_packet.packet_number);
    writer.put_u8(static_cast<uint8_t>(received_packet.type));
    writer.put_u8(static_cast<uint8_t>(status));
    ble_send_packet(BLE_Protocol::Message_Type::COMMAND_RESULT, writer);
}

}  // namespace Robot_Control