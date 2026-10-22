// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "ble_handler.h"
#include <string.h>
#include <zephyr/kernel.h>
#include "ble_payload_writer.h"
#include "ble_protocol_constants.h"
#include "ble_transfer_handler.h"
#include "control_loop.h"
#include "main_state_machine.h"
#include "robot_controller.h"

namespace Robot_Control
{

namespace
{

constexpr size_t COMMAND_QUEUE_CAPACITY = 4u;

struct Pending_Command
{
    BLE_Protocol::Message_Type type;
    uint8_t reserved;
    uint16_t payload_length;
    uint32_t packet_number;
    uint8_t payload[BLE_Protocol::MAX_PAYLOAD_SIZE];
};

K_MSGQ_DEFINE(
    command_queue, sizeof(Pending_Command), COMMAND_QUEUE_CAPACITY, alignof(Pending_Command));

}  // namespace

bool
BLE_Handler::enqueue_received_packet(BLE_Protocol::Received_Packet const& received_packet)
{
    if(received_packet.payload_length > BLE_Protocol::MAX_PAYLOAD_SIZE)
    {
        send_command_result(received_packet, BLE_Protocol::Command_Status::INVALID_LENGTH);
        return false;
    }

    Pending_Command pending_command = {
        .type           = received_packet.type,
        .reserved       = received_packet.reserved,
        .payload_length = received_packet.payload_length,
        .packet_number  = received_packet.packet_number,
    };
    if(received_packet.payload_length > 0u)
    {
        memcpy(pending_command.payload, received_packet.payload, received_packet.payload_length);
    }

    if(k_msgq_put(&command_queue, &pending_command, K_NO_WAIT) != 0)
    {
        send_command_result(received_packet, BLE_Protocol::Command_Status::INVALID_STATE);
        return false;
    }

    return true;
}

void
BLE_Handler::process_pending_commands(size_t maximum_commands)
{
    Pending_Command pending_command {};
    for(size_t command_index = 0u; command_index < maximum_commands; ++command_index)
    {
        if(k_msgq_get(&command_queue, &pending_command, K_NO_WAIT) != 0)
        {
            break;
        }

        BLE_Protocol::Received_Packet const received_packet = {
            .type           = pending_command.type,
            .reserved       = pending_command.reserved,
            .payload_length = pending_command.payload_length,
            .packet_number  = pending_command.packet_number,
            .payload        = pending_command.payload,
        };
        handle_received_packet(received_packet);
    }
}

void
BLE_Handler::handle_received_packet(BLE_Protocol::Received_Packet const& received_packet)
{
    auto status = BLE_Protocol::Command_Status::OK;

    BLE_Protocol::Payload_Reader reader(received_packet.payload, received_packet.payload_length);
    switch(received_packet.type)
    {
        case BLE_Protocol::Message_Type::STATE_COMMAND:
        {
            status = Main_State_Machine::instance().receive_command(reader);
            break;
        }
        case BLE_Protocol::Message_Type::DRIVE_COMMAND:
        {
            status = Robot_Controller::instance().set_drive_command(reader);
            break;
        }
        case BLE_Protocol::Message_Type::SET_CONTROL_MODE:
        {
            status = Control_Loop::instance().set_control_mode(reader);
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