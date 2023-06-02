/**
 ******************************************************************************
 * File Name          : DMBRxProtocolTask.hpp
 * Description        : Protocol task, specific to DMBRx
 ******************************************************************************
*/
#ifndef SOAR_DMBRXPROTOCOL_HPP_
#define SOAR_DMBRXPROTOCOL_HPP_
#include "ProtocolTask.hpp"
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "UARTTask.hpp"

/* Enums ------------------------------------------------------------------*/

/* Class ------------------------------------------------------------------*/
class DMBRxProtocolTask : public ProtocolTask
{
public:
    static DMBRxProtocolTask& Inst() {
        static DMBRxProtocolTask inst;
        return inst;
    }

    void InitTask();

    static void SendProtobufMessage(EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE>& writeBuffer, Proto::MessageID msgId)
    {
        Inst().ProtocolTask::SendProtobufMessage(writeBuffer, msgId);
    }

protected:
    static void RunTask(void* pvParams) { DMBRxProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    // These handlers will receive a buffer and size corresponding to a decoded message
    void HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    
    // Member variables

private:
    DMBRxProtocolTask();        // Private constructor
    DMBRxProtocolTask(const DMBRxProtocolTask&);                        // Prevent copy-construction
    DMBRxProtocolTask& operator=(const DMBRxProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_DMBRxPROTOCOL_HPP_
