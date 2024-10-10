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

//GPIO initialize
GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin;

/**
 * @brief Constructor for TemperatureControl
 */
TemperatureControl::TemperatureControl() : Task()
{
    // Initialize the TempControl array with AC units and target temperatures
	tempControl[0] = {TARGET_CONTROLS::AC1, 10, false, 0};  // AC1, target temperature 10, initially off, currentTemp
	tempControl[1] = {TARGET_CONTROLS::AC2, 20, false, 0};  // AC2, target temperature 20, initially off, currentTemp
}


/**
 * @brief Initialize the TemperatureControl
 */

void TemperatureControl::InitTask()
{
    // Make sure the task is not already initialized
//    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");
//
//    BaseType_t rtValue =
//        xTaskCreate((TaskFunction_t)TemperatureControl::RunTask,
//            (const char*)"TemperatureControl",
//            (uint16_t)TEMPERATURE_TASK_STACK_DEPTH_WORDS, //define
//            (void*)this,
//            (UBaseType_t)TEMPERATURE_TASK_RTOS_PRIORITY, //define
//            (TaskHandle_t*)&rtTaskHandle);
//
//    SOAR_ASSERT(rtValue == pdPASS, "TemperatureTask::InitTask() - xTaskCreate() failed");

	//Task setup


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
	        for (int i = 0; i<2; ++i){
	        	HandleCommand(cm); //would proccess the command
	        	//int currTemp = SampleThermocouple(i); //Read current temp from Termocouples -> given from GUI
	        	//need to store in the array

	        	if (currTemp > tempControl[i].targetTemperature){
	        		GPIO::AcStatus::On();
	        	}
	        	else{
	        		GPIO::AcStatus::OFF();
	        	}
	        }

	}
}

/*
 * @brief Handles command
 *
 */
void TemperatureControl::HandleCommand(Command& cm)
{
	//Switch for the GLOBAL_COMMAND
//	    switch (cm.GetCommand()) {
//	    case REQUEST_COMMAND: {
//	        HandleRequestCommand(cm.GetTaskCommand()); //Sends task specific request command to task request handler
//	        break;
//	    }
//	    case TASK_SPECIFIC_COMMAND: {
//	        break; //No task specific commands need
//	    }
//	    default:
//	        SOAR_PRINT("ThermocoupleTask - Received Unsupported Command {%d}\n", cm.GetCommand());//change
//	        break;
//	    }
//
//	    cm.Reset();

	//Set TargetTemp

}

/*
 * @brief Handles a Request Command
 *
 */
//void TemperatureControl::HandleRequestCommand(uint16_t taskCommand)
//{
//	//
//	switch (taskCommand) {
//	    case THERMOCOUPLE_REQUEST_NEW_SAMPLE: //Sample TC and store in class fields
//	    	SampleThermocouple();
//	        break;
//	    case THERMOCOUPLE_REQUEST_TRANSMIT: //Sending data to PI
//	        TransmitProtocolThermoData();
//	        break;
//	    case THERMOCOUPLE_REQUEST_DEBUG: //Output TC data
//	        ThermocoupleDebugPrint();
//	        break;
//	    default:
//	        SOAR_PRINT("UARTTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
//	        break;
//	    }
//}

/**
 * @brief This method receives the voltage reading through spi from the thermocouple readings
 */
void TemperatureControl::SampleThermocouple(Temp_Control& temp_p){
	//Will update and store the current temp received into the array

	temperature1 = ExtractTempurature(dataBuffer1); // Extract Temp is a palce holder where we will get TEMP from GUI -> implemented later

	temp_p.currTemperature = temperature1;

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
