/**
 ******************************************************************************
 * File Name          : PIRxProtocolTask.hpp
 * Description        : Protocol task, specific to PIRx
 ******************************************************************************
*/
#ifndef SOAR_PIRXPROTOCOL_HPP_
#define SOAR_PIRXPROTOCOL_HPP_
#include "ProtocolTask.hpp"
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "UARTTask.hpp"

/* Enums ------------------------------------------------------------------*/

/* Class ------------------------------------------------------------------*/
class PIRxProtocolTask : public ProtocolTask
{
public:
    static PIRxProtocolTask& Inst() {
        static PIRxProtocolTask inst;
        return inst;
    }

    void InitTask();

    static void SendProtobufMessage(EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE>& writeBuffer, Proto::MessageID msgId)
    {
        Inst().ProtocolTask::SendProtobufMessage(writeBuffer, msgId);
    }

protected:
    static void RunTask(void* pvParams) { PIRxProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    // These handlers will receive a buffer and size corresponding to a decoded message
    void HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    
    // Member variables

private:
    PIRxProtocolTask();        // Private constructor
    PIRxProtocolTask(const PIRxProtocolTask&);                        // Prevent copy-construction
    PIRxProtocolTask& operator=(const PIRxProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_PIRxPROTOCOL_HPP_
