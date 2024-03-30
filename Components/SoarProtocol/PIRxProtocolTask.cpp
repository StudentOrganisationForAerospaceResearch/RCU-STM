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
#include "LoadCellTask.hpp"
#include "GPIO.hpp"

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
    UART::RPI,
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

    if(msg.get_target() != Proto::Node::NODE_RCU) {
        return;
    }

    switch(msg.get_rcu_command().get_command_enum()) {
    case Proto::RcuCommand::Command::RCU_TARE_NOS1_LOAD_CELL: {
    	//NOTE: WORKS FOR TO NOS1 ONLY
        SOAR_PRINT("PROTO-INFO: Received RCU Tare NOS1 Load Cell Command\n");
        LoadCellTask::Inst().SendCommand(Command(REQUEST_COMMAND, (uint16_t)NOS1_LOADCELL_REQUEST_TARE));
        break;
    }
    case Proto::RcuCommand::Command::RCU_TARE_NOS2_LOAD_CELL: {
    	//NOTE: WORKS FOR TO NOS1 ONLY
        SOAR_PRINT("PROTO-INFO: Received RCU Tare NOS2 Load Cell Command\n");
        LoadCellTask::Inst().SendCommand(Command(REQUEST_COMMAND, (uint16_t)NOS2_LOADCELL_REQUEST_TARE));
        break;
    }
    case Proto::RcuCommand::Command::RCU_CALIBRATE_NOS1_LOAD_CELL: {
    	//NOTE: WORKS FOR TO NOS1 ONLY
        SOAR_PRINT("PROTO-INFO: Received RCU Calibrate NOS1 Load Cell Command\n");
        int32_t mass_mg = msg.get_rcu_command().get_command_param();
		LoadCellTask::Inst().SetCalibrationMassGrams((float)mass_mg / 1000);
		LoadCellTask::Inst().SendCommand(Command(REQUEST_COMMAND, NOS1_LOADCELL_REQUEST_CALIBRATE));
		break;
    }
    case Proto::RcuCommand::Command::RCU_CALIBRATE_NOS2_LOAD_CELL: {
    	//NOTE: WORKS FOR TO NOS1 ONLY
        SOAR_PRINT("PROTO-INFO: Received RCU Calibrate NOS2 Load Cell Command\n");
        int32_t mass_mg = msg.get_rcu_command().get_command_param();
		LoadCellTask::Inst().SetCalibrationMassGrams((float)mass_mg / 1000);
		LoadCellTask::Inst().SendCommand(Command(REQUEST_COMMAND, NOS2_LOADCELL_REQUEST_CALIBRATE));
		break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_AC1: {
        GPIO::SHEDAC::Off();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_AC1: {
        GPIO::SHEDAC::On();
        break;
    }
    case Proto::RcuCommand::Command::RCU_KILL_PAD_BOX1: {
        GPIO::PADBOX1::Kill();
        GPIO::PADBOX2::Kill();
        break;
    }
    case Proto::RcuCommand::Command::RCU_IGNITE_PAD_BOX1: {
        GPIO::PADBOX1::Ignite();
        GPIO::PADBOX2::Ignite();
        break;
    }
    case Proto::RcuCommand::Command::RCU_KILL_PAD_BOX2: {
        GPIO::PADBOX1::Kill();
        GPIO::PADBOX2::Kill();
        break;
    }
    case Proto::RcuCommand::Command::RCU_IGNITE_PAD_BOX2: {
        GPIO::PADBOX1::Ignite();
        GPIO::PADBOX2::Ignite();
        break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_PBV1: {
        GPIO::PBV1::Open();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_PBV1: {
        GPIO::PBV1::Close();
        break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_PBV2: {
        GPIO::PBV2::Open();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_PBV2: {
        GPIO::PBV2::Close();
        break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_PBV3: {
        GPIO::PBV3::Open();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_PBV3: {
        GPIO::PBV3::Close();
        break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_PBV4: {
        GPIO::PBV4::Open();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_PBV4: {
        GPIO::PBV4::Close();
        break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_SOL5: {
        GPIO::SOL5::Open();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_SOL5: {
        GPIO::SOL5::Close();
        break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_SOL6: {
        GPIO::SOL6::Open();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_SOL6: {
        GPIO::SOL6::Close();
        break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_SOL7: {
        GPIO::SOL7::Open();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_SOL7: {
        GPIO::SOL7::Close();
        break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_SOL8A: {
        GPIO::SOL8A::Open();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_SOL8A: {
        GPIO::SOL8A::Close();
        break;
    }
    case Proto::RcuCommand::Command::RCU_OPEN_SOL8B: {
        GPIO::SOL8B::Open();
        break;
    }
    case Proto::RcuCommand::Command::RCU_CLOSE_SOL8B: {
        GPIO::SOL8B::Close();
        break;
    }
    default:
        SOAR_PRINT("PIRxProtocolTask - Received Unsupported RCU commmand {%d}\n", msg.get_rcu_command());
		break;
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
