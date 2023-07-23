/**
 ******************************************************************************
 * File Name          : SOBRxProtocolTask.hpp
 * Description        : Protocol task, specific to SOBRx
 ******************************************************************************
*/
#ifndef SOAR_SOBRXPROTOCOL_HPP_
#define SOAR_SOBRXPROTOCOL_HPP_
#include "ProtocolTask.hpp"
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "UARTTask.hpp"
#include "SOBManager.hpp"



/* Class ------------------------------------------------------------------*/

class SOBRxProtocolTask : public ProtocolTask
{
public:
    static SOBRxProtocolTask& Inst() {
        static SOBRxProtocolTask inst;
        return inst;
    }

    void InitTask();

	static void SendProtobufMessage(EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE>& writeBuffer, Proto::MessageID msgId)
    {
        Inst().ProtocolTask::SendProtobufMessage(writeBuffer, msgId);
    }

    static void SendSOBCommand(Proto::SOBCommand::Command cmd)
    {
        Proto::CommandMessage cmdMsg;
        Proto::SOBCommand sobCmd;
        cmdMsg.set_source(Proto::Node::NODE_RCU);
        cmdMsg.set_target(Proto::Node::NODE_SOB);
        sobCmd.set_command_enum(cmd);
        cmdMsg.set_sob_command(sobCmd);
        EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
        cmdMsg.serialize(writeBuffer);
        SOBRxProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_COMMAND);
    }

protected:
    static void RunTask(void* pvParams) { SOBRxProtocolTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    // These handlers will receive a buffer and size corresponding to a decoded message
    void HandleProtobufCommandMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufControlMesssage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    void HandleProtobufTelemetryMessage(EmbeddedProto::ReadBufferFixedSize<PROTOCOL_RX_BUFFER_SZ_BYTES>& readBuffer);
    
    // Member variables

private:
    SOBRxProtocolTask();        // Private constructor
    SOBRxProtocolTask(const SOBRxProtocolTask&);                        // Prevent copy-construction
    SOBRxProtocolTask& operator=(const SOBRxProtocolTask&);            // Prevent assignment
};

#endif    // SOAR_SOBRxPROTOCOL_HPP_