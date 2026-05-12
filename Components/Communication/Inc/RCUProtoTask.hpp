/**
 ********************************************************************************
 * @file    RCUProtoTask.hpp
 * @author  Christy Guirguis
 * @date    May 12, 2026
 * @brief
 ********************************************************************************
 */

#ifndef RCUPROTOTASK_HPP_
#define RCUPROTOTASK_HPP_

/************************************
 * INCLUDES
 ************************************/
#include "ProtocolTask.hpp"
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "UARTTask.hpp"

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * CLASS DEFINITIONS
 ************************************/
class RCUProtocolTask : public ProtocolTask
{
public:
    static RCUProtocolTask& Inst() {
        static RCUProtocolTask inst;
        return inst;
    }

    void InitTask();

    static void SendProtobufMessage(EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE>& writeBuffer, Proto::MessageID msgId)
    {
        Inst().ProtocolTask::SendProtobufMessage(writeBuffer, msgId);
    }

protected:
    static void RunTask(void* pvParams) { RCUProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    // These handlers will receive a buffer and size corresponding to a decoded message
    void HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);


private:
    RCUProtocolTask();        // Private constructor
    RCUProtocolTask(const RCUProtocolTask&);                        // Prevent copy-construction
    RCUProtocolTask& operator=(const RCUProtocolTask&);            // Prevent assignment
};

/************************************
 * FUNCTION DECLARATIONS
 ************************************/

#endif /* RCUPROTOTASK_HPP_ */
