/**
 ******************************************************************************
 * File Name          : TelemetryTask.cpp
 * Description        : Primary telemetry task, default task for the system.
 ******************************************************************************
*/
#include "TelemetryTask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "PIRxProtocolTask.hpp"
#include "LoadCellTask.hpp"
#include "FlightTask.hpp"

/**
 * @brief Constructor for TelemetryTask
 */
TelemetryTask::TelemetryTask() : Task(TELEMETRY_TASK_QUEUE_DEPTH_OBJS)
{
    loggingDelayMs = TELEMETRY_DEFAULT_LOGGING_RATE_MS;
}

/**
 * @brief Initialize the TelemetryTask
 */
void TelemetryTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize telemetry task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)TelemetryTask::RunTask,
            (const char*)"TelemetryTask",
            (uint16_t)TELEMETRY_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TELEMETRY_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "TelemetryTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the Telemetry Task, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void TelemetryTask::Run(void* pvParams)
{
    while (1) {
        //Process all commands in queue this cycle
        Command cm;
		while (qEvtQueue->Receive(cm))
            HandleCommand(cm);

        osDelay(loggingDelayMs);
        RunLogSequence();
    }
}

/**
 * @brief Handles a command from the command queue
 * @param cm Command to handle
 */
void TelemetryTask::HandleCommand(Command& cm)
{
    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case TELEMETRY_CHANGE_PERIOD: {
        loggingDelayMs = (uint16_t)cm.GetTaskCommand();
	break;
    }
    default:
        SOAR_PRINT("TelemetryTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

/**
 * @brief Runs a full logging sample/send sequence.
 *        can assume this is called with a period of loggingDelayMs
 */
void TelemetryTask::RunLogSequence()
{
    SOAR_PRINT("GPIO Transmit...\n");

	//Relay status
    Proto::TelemetryMessage relayMsg;
    relayMsg.set_source(Proto::Node::NODE_RCU);
    relayMsg.set_target(Proto::Node::NODE_RCU);
    relayMsg.set_message_id((uint32_t)Proto::MessageID::MSG_TELEMETRY);
    Proto::RelayStatus relayStatus;
    relayStatus.set_ac1_open(GPIO::AC1::IsOpen());
    relayStatus.set_ac2_open(GPIO::AC2::IsOpen());
    relayStatus.set_pbv1_open(GPIO::PBV1::IsOpen());
    relayStatus.set_pbv2_open(GPIO::PBV2::IsOpen());
    relayStatus.set_pbv3_open(GPIO::PBV3::IsOpen());
    relayStatus.set_sol1_open(GPIO::SOL1::IsOpen());
    relayStatus.set_sol2_open(GPIO::SOL2::IsOpen());
    relayStatus.set_sol3_open(GPIO::SOL3::IsOpen());
    relayStatus.set_sol4_open(GPIO::SOL4::IsOpen());
    relayStatus.set_sol5_open(GPIO::SOL5::IsOpen());
    relayStatus.set_sol6_open(GPIO::SOL6::IsOpen());
    relayStatus.set_sol7_open(GPIO::SOL7::IsOpen());
    relayStatus.set_sol8a_open(GPIO::SOL8A::IsOpen());
    relayStatus.set_sol8b_open(GPIO::SOL8B::IsOpen());
    relayMsg.set_relay(relayStatus);

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> relayWriteBuffer;
    relayMsg.serialize(relayWriteBuffer);

    // Send the relay data
    PIRxProtocolTask::SendProtobufMessage(relayWriteBuffer, Proto::MessageID::MSG_TELEMETRY);

    //Padbox continuity status
    Proto::TelemetryMessage padBoxMsg;
    padBoxMsg.set_source(Proto::Node::NODE_RCU);
    padBoxMsg.set_target(Proto::Node::NODE_RCU);
    padBoxMsg.set_message_id((uint32_t)Proto::MessageID::MSG_TELEMETRY);
    Proto::PadBoxStatus padBoxStatus;
    padBoxStatus.set_cont1(GPIO::CONT_CK0::IsContinuous());
    padBoxStatus.set_cont2(GPIO::CONT_CK1::IsContinuous());
    padBoxMsg.set_padbox(padBoxStatus);

    EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> padBoxWriteBuffer;
    padBoxMsg.serialize(padBoxWriteBuffer);

    // Send the padbox continuity data
    PIRxProtocolTask::SendProtobufMessage(padBoxWriteBuffer, Proto::MessageID::MSG_TELEMETRY);

	// Load Cell
    LoadCellTask::Inst().SendCommand(Command(REQUEST_COMMAND, (uint16_t)LOADCELL_REQUEST_NEW_SAMPLE));
    LoadCellTask::Inst().SendCommand(Command(REQUEST_COMMAND, (uint16_t)LOADCELL_REQUEST_TRANSMIT));

    //TODO: Thermocouples
    //TODO: Pressure Transducers
}
