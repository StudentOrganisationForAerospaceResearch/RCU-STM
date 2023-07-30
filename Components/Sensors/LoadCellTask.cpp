/**
 ******************************************************************************
 * File Name          : LoadCellTask.cpp
 * Description        : Primary LoadCell task, default task for the system.
 ******************************************************************************
*/
#include <stdlib.h>
#include "LoadCellTask.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "PIRxProtocolTask.hpp"

/**
 * @brief Constructor for LoadCellTask
 */
LoadCellTask::LoadCellTask() : Task(LOADCELL_TASK_QUEUE_DEPTH_OBJS)
{
	two_fill_mass_sample = { 0 };
	calibration_mass_g = 0.0f;
}

/**
 * @brief Initialize the LoadCellTask
 */
void LoadCellTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)LoadCellTask::RunTask,
            (const char*)"LoadCellTask",
            (uint16_t)LOADCELL_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)LOADCELL_TASK_RTOS_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "LoadCellTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief Instance Run loop for the LoadCellTask, runs on scheduler start as long as the task is initialized.
 * @param pvParams RTOS Passed void parameters, contains a pointer to the object instance, should not be used
 */
void LoadCellTask::Run(void * pvParams)
{
	hx711_init(&nos1_loadcell, NOS1_LC_CLK_GPIO_Port, NOS1_LC_CLK_Pin , NOS1_LC_DATA_GPIO_Port, NOS1_LC_DATA_Pin);
	hx711_init(&nos2_loadcell, NOS2_LC_CLK_GPIO_Port, NOS2_LC_CLK_Pin , NOS2_LC_DATA_GPIO_Port, NOS2_LC_DATA_Pin);

	while (1) {

    	Command cm;

    	//Wait forever for a command
    	qEvtQueue->ReceiveWait(cm);

    	//Process the command
    	HandleCommand(cm);
    }
}


/**
 * @brief Handles a command
 * @param cm Command reference to handle
 */
void LoadCellTask::HandleCommand(Command& cm)
{
    //TODO: Since this task will stall for a few milliseconds, we may need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
    //TODO: Maybe a HandleEvtQueue instead that takes in the whole queue and eats the whole thing in order of non-blocking to blocking

	//NOTE: if receiving corrupt data from load cell task, consider disabling/enabling interrupts before/after reading load cell with bit banging
    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case REQUEST_COMMAND: {
        HandleRequestCommand(cm.GetTaskCommand());
        break;
    }
    case TASK_SPECIFIC_COMMAND: {
        break;
    }
    default:
        SOAR_PRINT("LoadCellTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}


/**
 * @brief Handles a Request Command
 * @param taskCommand The command to handle
 */
void LoadCellTask::HandleRequestCommand(uint16_t taskCommand)
{
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case NOS1_LOADCELL_REQUEST_TARE: {
    	LoadCellTare(&nos1_loadcell);
    	break;
    }
    case NOS1_LOADCELL_REQUEST_CALIBRATE: {
       	LoadCellCalibrate(&nos1_loadcell);
       	break;
    }
    case NOS2_LOADCELL_REQUEST_TARE: {
    	LoadCellTare(&nos2_loadcell);
    	break;
    }
    case NOS2_LOADCELL_REQUEST_CALIBRATE: {
       	LoadCellCalibrate(&nos2_loadcell);
       	break;
    }
    case LOADCELL_REQUEST_NEW_SAMPLE: {
    	SampleLoadCellData();
        break;
    }
    case LOADCELL_REQUEST_TRANSMIT: {
    	TransmitProtocolLoadCellData();
        break;
    }
    case LOADCELL_REQUEST_CALIBRATION_DEBUG: {
    	SOAR_PRINT("Load Cell offset NOS1: %d NOS2: %d\n", nos1_loadcell.offset, nos2_loadcell.offset);
    	SOAR_PRINT("Load Cell coef NOS1: %d.%d NOS2: %d.%d\n",
    				(int)nos1_loadcell.coef,
					abs(int(nos1_loadcell.coef * 1000) % 1000),
					(int)nos2_loadcell.coef,
					abs(int(nos2_loadcell.coef * 1000) % 1000));
    	SOAR_PRINT("Load Cell last used calibration weight %d.%d grams\n",
    			(int)calibration_mass_g, abs(int(calibration_mass_g * 1000) % 1000));
    	break;
    }
    case LOADCELL_REQUEST_DEBUG: {
        SOAR_PRINT("Load Cell read mass NOS1: %d.%d, NOS2: %d.%d grams\n",
        		(int)two_fill_mass_sample.nos1_mass_g,
				abs(int(two_fill_mass_sample.nos1_mass_g * 1000) % 1000),
				(int)two_fill_mass_sample.nos2_mass_g,
				abs(int(two_fill_mass_sample.nos2_mass_g * 1000) % 1000));
        break;
    }
    default:
        SOAR_PRINT("LoadCellTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

/**
 * @brief Sets up the load cell during tare. We need to call this before weighing
 * any mass. This is the second call.
 * @param none
 */
void LoadCellTask::LoadCellTare(hx711_t* loadcell)
{
	hx711_reset_coef_offset(loadcell);
	hx711_tare(loadcell, 10);
}
/**
 * @brief Calculates the calibration coefficient for calibration with a known mass.
 * This is the third call.
 * @param none
 */
void LoadCellTask::LoadCellCalibrate(hx711_t* loadcell)
{
	int32_t load_raw = hx711_value_ave(loadcell, 10);
	hx711_calibration(loadcell, loadcell->offset, load_raw, calibration_mass_g);
}

/**
 * @brief This samples the weight of any given mass after calibration. This is the
 * fourth call.
 * @param none
 */
void LoadCellTask::SampleLoadCellData()
{
	uint32_t nos1_ADCdata, nos2_ADCdata;
	hx711_weight(&nos1_loadcell, 10, nos1_ADCdata);
	two_fill_mass_sample.nos1_mass_g = nos1_ADCdata;
	hx711_weight(&nos2_loadcell, 10, nos2_ADCdata);
	two_fill_mass_sample.nos2_mass_g = nos2_ADCdata;
	two_fill_mass_sample.timestamp_ms = HAL_GetTick();
}

void LoadCellTask::TransmitProtocolLoadCellData()
{
    Proto::TelemetryMessage msg;
	msg.set_source(Proto::Node::NODE_RCU);
	msg.set_target(Proto::Node::NODE_RCU);
//	msg.set_message_id((uint32_t)Proto::MessageID::MSG_TELEMETRY);

	Proto::NOSLoadCell twofillSample;
	twofillSample.set_nos1_mass(two_fill_mass_sample.nos1_mass_g);
	twofillSample.set_nos2_mass(two_fill_mass_sample.nos2_mass_g);
	msg.set_nos(twofillSample);

	EmbeddedProto::WriteBufferFixedSize<DEFAULT_PROTOCOL_WRITE_BUFFER_SIZE> writeBuffer;
	msg.serialize(writeBuffer);

    // Send the load cell data
    PIRxProtocolTask::SendProtobufMessage(writeBuffer, Proto::MessageID::MSG_TELEMETRY);
}
