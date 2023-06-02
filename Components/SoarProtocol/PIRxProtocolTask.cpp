/**
 ******************************************************************************
 * File Name          : PIRxProtocolTask.hpp
 * Description        : Protocol task, specific to PIRx UART Line
 ******************************************************************************
*/
#include <SoarProtocol/PIRxProtocolTask.hpp>
#include "FlightTask.hpp"
#include "ReadBufferFixedSize.h"
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
void PIRxProtocolTask::HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer)
{
    Proto::CommandMessage msg;
    msg.deserialize(readBuffer);

    // Verify the source node, echo it if it does not have PIRx as the target (?)
    if (msg.get_source() == Proto::Node::NODE_RCU)
        return;

    // If the message does not have []
    if ()
        return;

    SOAR_PRINT("PROTO-INFO: Received PIRx Command Message");
    }

}

/**
 * @brief Handle a control message
 */
void PIRxProtocolTask::HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer)
{

}

/**
 * @brief Handle a telemetry message (unused?)
 */
void PIRxProtocolTask::HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer)
{

}
