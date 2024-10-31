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
#include "SystemDefines.hpp"
#include "Task.hpp"

//GPIO initialize


// Initialize the TempControl array with AC units and target temperatures - make static array
static Temp_Control tempControl[2] = {
    {TARGET_CONTROLS::AC1, 10, false, 0, LED_1_GPIO_Port, LED_1_Pin},  // AC1, target temperature 10, initially off, currentTemp
    {TARGET_CONTROLS::AC2, 20, false, 0, LED_1_GPIO_Port, LED_1_Pin}   // AC2, target temperature 20, initially off, currentTemp
};
/**
 * @brief Constructor for TemperatureControl
 */

TemperatureControl::TemperatureControl() : Task(){

}

/**
 * @brief Initialize the TemperatureControl
 */

void TemperatureControl::InitTask()
{
//     Make sure the task is not already initialized
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
void TemperatureControl::Run(void* pvParams){

	while (1) {
		Command cm;

		//Wait forever for a command
		if (qEvtQueue->Receive(cm, 1000)) {
			HandleCommand(cm);
		}
		else {
			// Update target state depending on temp
			for (Temp_Control target : tempControl){

				//int currTemp = SampleThermocouple(i); //Read current temp from Termocouples -> given from GUI
				//need to store in the array

				if (target.currTemperature > target.targetTemperature){
					HAL_GPIO_WritePin(target.targetPinPort, target.targetPin, GPIO_PIN_SET);
				}
				else{
					HAL_GPIO_WritePin(target.targetPinPort, target.targetPin, GPIO_PIN_RESET);
				}
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
	    switch (cm.GetCommand()) {

	    case TASK_SPECIFIC_COMMAND: {
	    	HandleTaskCommand(cm.GetTaskCommand());
	    	break; //No task specific commands need
	    }
	    default:
	        SOAR_PRINT("ThermocoupleTask - Received Unsupported Command {%d}\n", cm.GetCommand());//change
	        break;
	    }

	    cm.Reset();

	//Set TargetTemp -> user should be able to change through the GUI

}

/*
 * @brief Handles a Request Command
 *
 */
void TemperatureControl::HandleTaskCommand(uint16_t taskCommand)
{
	//
	switch (taskCommand) {
	    case SET_TARGET_TEMP: {
	    	SetTargetTemp(); //args must be fixed
	    }

	    case SET_TARGET_STATE: {
	    	SetTargetState();
	    }

	    case SET_CURRENT_TEMP: {
			SetCurrentTemp();
	    }
	    default:
	        SOAR_PRINT("UARTTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
	        break;
	    }
}

void SetTargetTemp(TARGET_CONTROLS Target, uint8_t Target_Temp){
	// for everything in the target array
	// if target == target
	// then set

	for (Temp_Control targetSettings : tempControl) {
	    if (targetSettings.acUnit == Target) {
	        targetSettings.targetTemperature = Target_Temp;
	    }
	}
}

void SetCurrentTemp(TARGET_CONTROLS Target, uint8_t tempReceived){
	// for everything in the target array
	// if tempReceived == tempReceived
	// then set

	for (Temp_Control targetSettings : tempControl) {
	    if (targetSettings.acUnit == Target) {
	        targetSettings.currTemperature = tempReceived;
	    }
	}
}

void SetTargetState(TARGET_CONTROLS Target, bool currentState){
	for (Temp_Control targetSettings : tempControl) {
		    if (targetSettings.acUnit == Target) {
		        targetSettings.isOn = true;
		    }

		    else{
		    	targetSettings.isOn = false;
		    }
	}

}
