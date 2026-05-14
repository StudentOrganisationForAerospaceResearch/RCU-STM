/**
 ********************************************************************************
 * @file    RCUProtoTask.cpp
 * @author  Christy Guirguis
 * @date    May 12, 2026
 * @brief
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "RCUProtoTask.hpp"
#include "FlightTask.hpp"
#include "ReadBufferFixedSize.h"

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/

/************************************
 * VARIABLES
 ************************************/

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

/************************************
 * FUNCTION DEFINITIONS
 ************************************/

/**
 * @brief Initialize the RCUProtocolTask
 */
void RCUProtocolTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize RCUProtocolTask twice");

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
RCUProtocolTask::RCUProtocolTask() : ProtocolTask(
        Proto::Node::NODE_FSB,
        UART::SOB,
        UART_TASK_COMMAND_SEND_PBB)
{
}

/**
 * @brief Handle a command message
 */
void RCUProtocolTask::HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
    Proto::CommandMessage msg;
    msg.deserialize(readBuffer);

    // Verify the source and target nodes, if they aren't as expected, do nothing
    if (msg.get_source() != Proto::Node::NODE_FSB || msg.get_target() != Proto::Node::NODE_FCB)
        return;

    // If the message does not have a FSB command, do nothing
    if (!msg.has_fsb_command()) // idk if its supposed to be an fsb command
        return;



}

/**
 * @brief Handle a control message
 */
void RCUProtocolTask::HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
   ;
}


/**
 * @brief Handle a control message
 */
void RCUProtocolTask::HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer)
{
   ;
}
