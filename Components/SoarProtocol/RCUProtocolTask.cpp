/**
 ******************************************************************************
 * File Name          : RCUProtocolTask.hpp
 * Description        : Protocol task, specific to RCU
 ******************************************************************************
*/
#include <SoarProtocol/RCUProtocolTask.hpp>
#include "FlightTask.hpp"
#include "ReadBufferFixedSize.h"

/**
 * @brief Initialize the RCUProtocolTask
 */
void RCUProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Protocol task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)RCUProtocolTask::RunTask,
            (const char*)"ProtocolTask",
            (uint16_t)TASK_PROTOCOL_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_PROTOCOL_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeded
    SOAR_ASSERT(rtValue == pdPASS, "ProtocolTask::InitTask - xTaskCreate() failed");
}

/**
 * @brief Default constructor
 */
RCUProtocolTask::RCUProtocolTask() : ProtocolTask(Proto::Node::NODE_RCU)
{
}

/**
 * @brief Handle a command message
 */
void RCUProtocolTask::HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer)
{
    Proto::CommandMessage msg;
    msg.deserialize(readBuffer);

    // Verify the source and target nodes, echo it if it does not have RCU as the target
    if (msg.get_source() != Proto::Node::NODE_RCU || msg.get_target() != Proto::Node::NODE_RCU)
        return;

    // If the message does not have a RCU command, do nothing
    if (!msg.has_rcu_command())
        return;

    SOAR_PRINT("PROTO-INFO: Received RCU Command Message");

    // Process the db command
    switch (msg.get_rcu_command().get_command_enum())
    {
    case Proto::DMBCommand::Command::RSC_ANY_TO_ABORT:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_ANY_TO_ABORT));
        break;
    case Proto::DMBCommand::Command::RSC_OPEN_VENT:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_OPEN_VENT));
        break;
    case Proto::DMBCommand::Command::RSC_CLOSE_VENT:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_CLOSE_VENT));
        break;
    case Proto::DMBCommand::Command::RSC_OPEN_DRAIN:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_OPEN_DRAIN));
        break;
    case Proto::DMBCommand::Command::RSC_CLOSE_DRAIN:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_CLOSE_DRAIN));
        break;
    case Proto::DMBCommand::Command::RSC_MEV_CLOSE:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_MEV_CLOSE));
        break;
    case Proto::DMBCommand::Command::RSC_GOTO_FILL:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_GOTO_FILL));
        break;
    case Proto::DMBCommand::Command::RSC_ARM_CONFIRM_1:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_ARM_CONFIRM_1));
        break;
    case Proto::DMBCommand::Command::RSC_ARM_CONFIRM_2:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_ARM_CONFIRM_2));
        break;
    case Proto::DMBCommand::Command::RSC_GOTO_ARM:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_GOTO_ARM));
        break;
    case Proto::DMBCommand::Command::RSC_GOTO_PRELAUNCH:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_GOTO_PRELAUNCH));
        break;
    case Proto::DMBCommand::Command::RSC_POWER_TRANSITION_ONBOARD:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_POWER_TRANSITION_ONBOARD));
        break;
    case Proto::DMBCommand::Command::RSC_POWER_TRANSITION_EXTERNAL:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_POWER_TRANSITION_EXTERNAL));
        break;
    case Proto::DMBCommand::Command::RSC_GOTO_IGNITION:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_GOTO_IGNITION));
        break;
    case Proto::DMBCommand::Command::RSC_IGNITION_TO_LAUNCH:
        FlightTask::Inst().SendCommand(Command(CONTROL_ACTION, (uint16_t)RSC_IGNITION_TO_LAUNCH));
        break;
    default:
        break;
    }

}

/**
 * @brief Handle a control message
 */
void RCUProtocolTask::HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer)
{

}

/**
 * @brief Handle a telemetry message
 */
void RCUProtocolTask::HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer)
{

}
