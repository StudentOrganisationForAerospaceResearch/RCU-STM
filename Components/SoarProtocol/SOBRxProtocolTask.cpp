/**
 ******************************************************************************
 * File Name          : SOBRxProtocolTask.hpp
 * Description        : Protocol task, specific to SOBRx UART Line
 ******************************************************************************
*/
#include "SOBRxProtocolTask.hpp"
#include "FlightTask.hpp"
#include "ReadBufferFixedSize.h"
#include "PIRxProtocolTask.hpp"
#include "SOBRxProtocolTask.hpp"
#include "UARTTask.hpp"


/**
 * @brief Initialize the SOBRxProtocolTask
 */
void SOBRxProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Protocol task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)SOBRxProtocolTask::RunTask,
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
SOBRxProtocolTask::SOBRxProtocolTask() : ProtocolTask(Proto::Node::NODE_SOB,
    SystemHandles::UART_SOB,
    UART_TASK_COMMAND_SEND_SOB)
{
}

/**
 * @brief Handle a command message
 */
void SOBRxProtocolTask::HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{


    Proto::CommandMessage msg;
    msg.deserialize(readBuffer);

    // Verify the target node, if it isn't as expected, do nothing
    if (msg.get_target() != Proto::Node::NODE_SOB)
        return;
     // If the message does not have a SOB command, do nothing
    if (!msg.has_sob_command())
        return;

    SOAR_PRINT("PROTO-INFO: Received SOB Command Message\n");

    // Process the SOB command
    switch (msg.get_sob_command().get_command_enum())
    {
    case Proto::SOBCommand::Command::SOB_FAST_SAMPLE_IR: // EOF command from SOB
    {
       SOBManager::Inst().ConfirmEOF();
       break;
    }
    default:
       break;
    }
}

/**
 * @brief Handle a control message
 */
void SOBRxProtocolTask::HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
    //rewrap into a write buffer var because readBuffer and writeBuffer are not interchangeable
    Proto::ControlMessage msg;
    msg.deserialize(readBuffer);

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
    msg.serialize(writeBuffer);

    PIRxProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_CONTROL);
}

/**
 * @brief Handle a telemetry message
 */
void SOBRxProtocolTask::HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
    //rewrap into a write buffer var because readBuffer and writeBuffer are not interchangeable
    Proto::TelemetryMessage msg;
    msg.deserialize(readBuffer);

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
    msg.serialize(writeBuffer);

    PIRxProtocolTask::Inst().SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_TELEMETRY);
}
