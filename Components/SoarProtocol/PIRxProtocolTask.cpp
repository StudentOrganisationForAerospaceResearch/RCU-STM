/**
 ******************************************************************************
 * File Name          : PIRxProtocolTask.hpp
 * Description        : Protocol task, specific to PIRx UART Line
 ******************************************************************************
*/
#include "PIRxProtocolTask.hpp"
#include "FlightTask.hpp"
#include "ReadBufferFixedSize.h"
#include "SOBRxRepeaterTask.hpp"
#include "DMBRxProtocolTask.hpp"
#include "UARTTask.hpp"

/**
 * @brief Initialize the PIRxProtocolTask
 */
void PIRxProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Protocol task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)PIRxProtocolTask::RunTask,
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
PIRxProtocolTask::PIRxProtocolTask() : ProtocolTask(Proto::Node::NODE_RCU, 
    SystemHandles::UART_PI,
    UART_TASK_COMMAND_SEND_PI)
{
}

/**
 * @brief Handle a command message
 */
void PIRxProtocolTask::HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
    Proto::CommandMessage msg;
    msg.deserialize(readBuffer);

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
    msg.serialize(writeBuffer);

    //Send to relevant destination
    if(msg.get_target() == Proto::Node::NODE_DMB || msg.get_target() == Proto::Node::NODE_PBB) {
        DMBRxProtocolTask::Inst().SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_COMMAND);
        return;
    }

    if(msg.get_target() == Proto::Node::NODE_SOB) {
        SOBRxRepeaterTask::Inst().SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_COMMAND);
        return;
    }

}

/**
 * @brief Handle a control message
 */
void PIRxProtocolTask::HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
    //rewrap into a write buffer var because readBuffer and writeBuffer are not interchangeable
    Proto::TelemetryMessage msg;
    msg.deserialize(readBuffer);

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
    msg.serialize(writeBuffer);

    //Send to relevant destination
    if(msg.get_target() == Proto::Node::NODE_DMB || msg.get_target() == Proto::Node::NODE_PBB) {
        DMBRxProtocolTask::Inst().SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_CONTROL);
        return;
    }

    if(msg.get_target() == Proto::Node::NODE_SOB) {
        SOBRxRepeaterTask::Inst().SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_CONTROL);
        return;
    }
}

/**
 * @brief Handle a telemetry message (unused?)
 */
void PIRxProtocolTask::HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{

}
