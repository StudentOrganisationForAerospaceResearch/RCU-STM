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
	//Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize flight task twice");

    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)TemperatureControl::RunTask,
            (const char*)"TemperatureControl",
            (uint16_t)TEMPERATURE_TASK_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TEMPERATURE_TASK_RTOS_PRIORITY,
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
	    	for (Temp_Control& target : tempControl) {
	    		// The for loop iterates over all instances of Temp_Control.
	    		// It ensures the SetTargetTemp function is called for each air conditioning unit (e.g., AC1, AC2, etc.),
	    		// dynamically updating their respective target temperatures as needed.
	    	    SetTargetTemp(target.acUnit, target.targetTemperature);
	    	}
	    }

	    case SET_TARGET_STATE: {
	    	for(Temp_Control& target : tempControl){
	    		SetTargetState(target.acUnit, target.currTemperature);
	    	}
	    }

	    case SET_CURRENT_TEMP: {
	    	for(Temp_Control& target : tempControl){
	    		SetCurrentTemp(target.acUnit, target.isOn);
	    	}
	    }
	    default:
	        SOAR_PRINT("UARTTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
	        break;
	    }
}

void TemperatureControl::SetTargetTemp(TARGET_CONTROLS Target, uint8_t Target_Temp){ // Data_command, setcurrenttemp into the Command IMUData -> sent as a struct then you would destruct it
	// Iterate through each element in the target array.
	// If the current target matches the specified target, update its value accordingly

	for (Temp_Control targetSettings : tempControl) {
	    if (targetSettings.acUnit == Target) {
	        targetSettings.targetTemperature = Target_Temp;
	    }
	}
}

void TemperatureControl::SetCurrentTemp(TARGET_CONTROLS Target, uint8_t tempReceived){
	// Iterate through each element in the target array.
	// If the received temperature matches the current temperature, update the corresponding value.


	for (Temp_Control targetSettings : tempControl) {
	    if (targetSettings.acUnit == Target) {
	        targetSettings.currTemperature = tempReceived;
	    }
	}

	//Command;
}

void TemperatureControl::SetTargetState(TARGET_CONTROLS Target, bool currentState){
	// Iterate through each element in the target array.
	// If the current target matches the specified target, set its state to "on." Otherwise, set its state to "off."


	for (Temp_Control targetSettings : tempControl) {
		    if (targetSettings.acUnit == Target) {
		        targetSettings.isOn = true;
		    }

		    else{
		    	targetSettings.isOn = false;
		    }
	}

}
