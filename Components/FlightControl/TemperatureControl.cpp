/**
 **********************************************************************************
 * File Name          : TemperatureControl.cpp
 * Description        : This file is able to control the AC/Cooling unit by
 * 						reading the temperature from the thermocouples and when
 * 						it reaches a specified temperature that is too high, the
 * 						cooling unit is turned on, and turned off when the desired
 * 						temperature is reached.
 **********************************************************************************
*/
#include <TemperatureControl.hpp>
#include "GPIO.hpp"
#include "GPIO.hpp"
#include "SystemDefines.hpp"
#include "Task.hpp"

/**
 * @brief Constructor for TemperatureControl
 */
TemperatureControl::TemperatureControl() : Task(TEMPERATURE_TASK_QUEUE_DEPTH_OBJS)
{
}

/**
 * @brief Initialize the TemperatureControl
 */

void TemperatureControl::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)TemperatureControl::RunTask,
            (const char*)"TemperatureControl",
            (uint16_t)TEMPERATURE_TASK_STACK_DEPTH_WORDS, //define
            (void*)this,
            (UBaseType_t)TEMPERATURE_TASK_RTOS_PRIORITY, //define
            (TaskHandle_t*)&rtTaskHandle);

    SOAR_ASSERT(rtValue == pdPASS, "TemperatureTask::InitTask() - xTaskCreate() failed");
}

/*
 * @brief Runs loop, waits for a command
 *
 */
void TemperatureControl::Run(void* pvParams)
{
	while (1) {
	        Command cm;

	        //Wait forever for a command
	        qEvtQueue->ReceiveWait(cm);

	        //Process the command
	        HandleCommand(cm);
	}
}

/*
 * @brief Handles command
 *
 */
void TemperatureControl::HandleCommand(Command& cm)
{
	//Switch for the GLOBAL_COMMAND
	    switch (cm.GetCommand()) {
	    case REQUEST_COMMAND: {
	        HandleRequestCommand(cm.GetTaskCommand()); //Sends task specific request command to task request handler
	        break;
	    }
	    case TASK_SPECIFIC_COMMAND: {
	        break; //No task specific commands need
	    }
	    default:
	        SOAR_PRINT("ThermocoupleTask - Received Unsupported Command {%d}\n", cm.GetCommand());//change
	        break;
	    }

	    cm.Reset();
}

/*
 * @brief Handles a Request Command
 *
 */
void TemperatureControl::HandleRequestCommand(uint16_t taskCommand)
{
	switch (taskCommand) {
	    case THERMOCOUPLE_REQUEST_NEW_SAMPLE: //Sample TC and store in class fields
	    	SampleThermocouple();
	        break;
//	    case THERMOCOUPLE_REQUEST_TRANSMIT: //Sending data to PI
//	        TransmitProtocolThermoData();
//	        break;
//	    case THERMOCOUPLE_REQUEST_DEBUG: //Output TC data
//	        ThermocoupleDebugPrint();
//	        break;
	    default:
	        SOAR_PRINT("UARTTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
	        break;
	    }
}

/**
 * @brief This method receives the voltage reading through spi from the thermocouple readings
 */
void TemperatureControl::SampleThermocouple(){

	temperature1 = ExtractTempurature(dataBuffer1);

	if(temperature1 >= 30){ //random number for now
		//part that would send command to turn the cooling unit on
		// AcStatus = 1; -> 1 is true -> which will switch the state on -> need to define AC status variable
		//> either use this as the bool, where when true, relays to turn it off
		GPIO::AcStatus::On();
	}

	else{
		//part that would detect that if the temperature read from the thermocouple is too low, the cooling
		//unit would be turned off
		GPIO::AcStatus::Off();
		//AcStatus = 0; -> either use this as the bool, where when true, relays to turn it off
	}
}


/**TAKEN FROM THERMOCOUPLE - NEEDS TO BE EDITED/CHANGED
 * @brief This method converts the thermocouple data buffer information to readable a temperature
 * takes the array containing temperature data, returns a temperature value
 */
int16_t TemperatureControl::ExtractTempurature(uint8_t temperatureData[]) //TAKEN FROM THERMOCOUPLE - NEEDS TO BE EDITED/CHANGED
{
	//read/extract value from packet
	return 0;
}
