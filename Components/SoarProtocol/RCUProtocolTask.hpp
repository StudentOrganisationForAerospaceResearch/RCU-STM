/**
 ******************************************************************************
 * File Name          : RCUProtocolTask.hpp
 * Description        : Protocol task, specific to RCU
 ******************************************************************************
*/
#ifndef SOAR_RCUPROTOCOL_HPP_
#define SOAR_RCUPROTOCOL_HPP_
#include "ProtocolTask.hpp"
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "UARTTask.hpp"

/* Enums ------------------------------------------------------------------*/

/* Class ------------------------------------------------------------------*/
class RCUProtocolTask : public ProtocolTask
{
public:
    static RCUProtocolTask& Inst() {
        static RCUProtocolTask inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { RCUProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    // These handlers will receive a buffer and size corresponding to a decoded message
    void HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer);
    void HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer);
    void HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES> readBuffer);
    
    // Member variables

private:
    RCUProtocolTask();        // Private constructor
    RCUProtocolTask(const RCUProtocolTask&);                        // Prevent copy-construction
    RCUProtocolTask& operator=(const RCUProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_RCUPROTOCOL_HPP_
