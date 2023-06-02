/**
 ******************************************************************************
 * File Name          : DMBRxProtocolTask.hpp
 * Description        : Protocol task, specific to DMBRx UART Line
 ******************************************************************************
*/
#include <SoarProtocol/DMBRxProtocolTask.hpp>
#include "FlightTask.hpp"
#include "ReadBufferFixedSize.h"

/**
 * @brief Initialize the DMBRxProtocolTask
 */
void DMBRxProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Protocol task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)DMBRxProtocolTask::RunTask,
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
DMBRxProtocolTask::DMBRxProtocolTask() : ProtocolTask(Proto::Node::NODE_DMBRx)
{
}

/**
 * @brief Handle a command message
 */
void DMBRxProtocolTask::HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer)
{
    Proto::CommandMessage msg;
    msg.deserialize(readBuffer);

    // Verify the source and target nodes, echo it if it does not have DMBRx as the target
    if (msg.get_source() != Proto::Node::NODE_DMBRx || msg.get_target() != Proto::Node::NODE_DMBRx)
        return;

    // If the message does not have a DMBRx command, do nothing
    if (!msg.has_DMBRx_command())
        return;

    SOAR_PRINT("PROTO-INFO: Received DMBRx Command Message");
    }

}

/**
 * @brief Handle a control message
 */
void DMBRxProtocolTask::HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer)
{

}

/**
 * @brief Handle a telemetry message
 */
void DMBRxProtocolTask::HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer)
{

}
