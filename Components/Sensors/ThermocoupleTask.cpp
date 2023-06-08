/**
  ******************************************************************************
  * File Name          : ThermocoupleTask.cpp
  * Description        :
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "ThermocoupleTask.hpp"
#include "main.h"
#include "DebugTask.hpp"
#include "Task.hpp"


/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Values should not be modified, non-const due to HAL and C++ strictness) ---*/
constexpr int CMD_TIMEOUT = 150; //TODO: Taken from IMU not sure if it needs to be different

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Default constructor
 */
ThermocoupleTask::ThermocoupleTask() : Task(TASK_THERMOCOUPLE_QUEUE_DEPTH_OBJS)
{
	//Data is stored locally in object not using MALOC
}

/**
 * @brief Creates a task for the FreeRTOS Scheduler
 */
void ThermocoupleTask::InitTask() //TODO: ETHAN NOTE: IDK IF WE ARE STILL DOING THIS
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize Thermocouple task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)ThermocoupleTask::RunTask,
            (const char*)"ThermocoupleTask",
            (uint16_t)TASK_THERMOCOUPLE_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_THERMOCOUPLE_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeeded
    SOAR_ASSERT(rtValue == pdPASS, "ThermocoupleTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief ThermocoupleTask run loop
 * @param pvParams Currently unused task context
 */
void ThermocoupleTask::Run(void * pvParams)
{
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
void ThermocoupleTask::HandleCommand(Command& cm)
{

	//ETHAN NOTE: IDK WHAT TODO IS ABOUT
    //TODO: Since this task will stall for a few milliseconds, we may need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
    //TODO: Maybe a HandleEvtQueue instead that takes in the whole queue and eats the whole thing in order of non-blocking to blocking

    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case REQUEST_COMMAND: {
        HandleRequestCommand(cm.GetTaskCommand()); //Sends task specific request command to task request handler
    }
    case TASK_SPECIFIC_COMMAND: {
        break; //No task specific commands need
    }
    default:
        SOAR_PRINT("ThermocoupleTask - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

/**
 * @brief Handles a Request Command
 * @param taskCommand The command to handle
 */
void ThermocoupleTask::HandleRequestCommand(uint16_t taskCommand)
{
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case THERMOCOUPLE_REQUEST_NEW_SAMPLE:
    	SampleThermocouple();
        break;
    case THERMOCOUPLE_REQUEST_TRANSMIT: //This is where we will actually be sending data
        SOAR_PRINT("Stubbed: Thermocouple task transmit not implemented\n");
        break;
    case THERMOCOUPLE_REQUEST_DEBUG: //Temporary data debug sender
        SOAR_PRINT("\n-- Thermocouple Data --\n");
        SampleThermocouple();
        break;
    default:
        SOAR_PRINT("UARTTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}

/**
 * @brief This method receives the voltage reading through spi from the thermocouple readings
 */

void ThermocoupleTask::SampleThermocouple()
{
	/*DATA FROM MAX31855KASA+T ------------------------------------------------------

	32 bits Memory Map

		D31-D18 : Thermocoupler Temperature Data

			D31 : Sign bit

			D30-D18 : Temperature Value (2's complement) from 2^10 to 2^-2

		D17 : Reserved Bit

		D16 : Fault (if high shows fault is detected, more specific fault messages at D2 - D0)

		D15-D4 :  Internal Temperature Data (reference junction temperature)

			D15 : Sign bit

			D14-D4 : Temperature Value (2's complement) from 2^6 to 2^-4

		D3 : Reserved

		D2-D0 : Fault Detection Bits

			D2 : SCV Fault (displays high if TC shorts to Vcc)

			D1 : SCG Fault (displays high if TC shorts to GND)

			D0 : Thermocouple has no Connection (displays high)

	*///------------------------------------------------------------------------------

	//Storable Data ------------------------------------------------------------------------------

	uint8_t dataBuffer1[4] = {0};
	uint8_t dataBuffer5[5] = {0};
	//See Above bit mem-map


	uint8_t Error1=0;// Thermocouple Connection acknowledge Flag
	uint32_t sign1=0;
	int Temp1=0;

	//Read ---------------------------------------------------------------------------------------
	HAL_GPIO_WritePin(TC1_NCS_GPIO_Port, TC1_NCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TC2_NCS_GPIO_Port, TC2_NCS_Pin, GPIO_PIN_SET);


    //Read From Thermocouple 1 first
	HAL_GPIO_WritePin(TC1_NCS_GPIO_Port, TC1_NCS_Pin, GPIO_PIN_RESET); //begin read with CS pin low
	HAL_Delay(10);
	HAL_SPI_Receive(SystemHandles::SPI_Thermocouple1, dataBuffer5, 5, 1000); //Fill the data buffer with data from TC1
	HAL_Delay(10);
	HAL_GPIO_WritePin(TC1_NCS_GPIO_Port, TC1_NCS_Pin, GPIO_PIN_SET); //end read with setting CS pin to high again

	for(int i = 0; i<5; i++){
		if(i!=4)
		dataBuffer1[i] = dataBuffer5[i];
//		SOAR_PRINT("databufferTC1[%d] are: %d\n",i, dataBuffer5[i]);
	}

	double temp1_value = 0;

	Error1=dataBuffer1[3]&0x07;								  // Error Detection
	sign1=(dataBuffer1[0]&(0x80))>>7;							  // Sign Bit calculation

	if(dataBuffer1[1] & 0x01){								  // Returns Error Number
		if(Error1 & 0x01){
			SOAR_PRINT("Thermocouple 1 is not connected\n");
		}
		if(Error1 & 0x02){
			SOAR_PRINT("Thermocouple 1 is shorted to GND\n");
		}
		if(Error1 & 0x04){
			SOAR_PRINT("Thermocouple 1 is shorted to VCC\n");
		}
		SOAR_PRINT("\n");
		//temp1_value = (-1*(dataBuffer1[3] & 0x07));
	}
	else if(sign1==1){									  // Negative Temperature
		Temp1 = (dataBuffer1[0] << 6) | (dataBuffer1[1] >> 2);
		Temp1&=0b01111111111111;
		Temp1^=0b01111111111111;
		temp1_value = (double)-Temp1/4;
	}
	else												  // Positive Temperature
	{
		Temp1 = (dataBuffer1[0] << 6) | (dataBuffer1[1] >> 2);
		temp1_value = ((double)Temp1 / 4);
	}

	if(!(dataBuffer1[1] & 0x01)){
		temp1_value = temp1_value*100-320; //room temp should've been 22.5, it was reading ambient 25.7
		SOAR_PRINT("Thermocouple 1 is reading %d.%d C \n\n" , (int)temp1_value/100, (uint8_t)(int)temp1_value%100);
	}

	uint8_t dataBuffer2[4] = {0};
	dataBuffer5[5] = {0};
	//See Above bit mem-map

	uint8_t Error2=0;// Thermocouple Connection acknowledge Flag
	uint32_t sign2=0;
	int Temp2=0;

	//Read ---------------------------------------------------------------------------------------
	HAL_GPIO_WritePin(TC1_NCS_GPIO_Port, TC1_NCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TC2_NCS_GPIO_Port, TC2_NCS_Pin, GPIO_PIN_SET);

	//Read From Thermocouple 1 first
	HAL_GPIO_WritePin(TC2_NCS_GPIO_Port, TC2_NCS_Pin, GPIO_PIN_RESET); //begin read with CS pin low
	HAL_Delay(10);
	HAL_SPI_Receive(SystemHandles::SPI_Thermocouple1, dataBuffer5, 5, 1000); //Fill the data buffer with data from TC1
	HAL_Delay(10);
	HAL_GPIO_WritePin(TC2_NCS_GPIO_Port, TC2_NCS_Pin, GPIO_PIN_SET); //end read with setting CS pin to high again

	for(int i = 0; i<5; i++){
		if(i!=4)
		dataBuffer2[i] = dataBuffer5[i];
		//SOAR_PRINT("databufferTC1[%d] are: %d\n",i, dataBuffer5[i]);
	}

	double temp2_value = 0;

	Error2=dataBuffer2[3]&0x07;								  // Error Detection
	sign2=(dataBuffer2[0]&(0x80))>>7;							  // Sign Bit calculation

	if(dataBuffer2[1] & 0x01){								  // Returns Error Number
		if(Error2 & 0x01){
			SOAR_PRINT("Thermocouple 2 is not connected\n");
		}
		if(Error2 & 0x02){
			SOAR_PRINT("Thermocouple 2 is shorted to GND\n");
		}
		if(Error2 & 0x04){
			SOAR_PRINT("Thermocouple 2 is shorted to VCC\n");
		}
		SOAR_PRINT("\n");
		//temp2_value = (-1*(dataBuffer2[3] & 0x07));
	}
	else if(sign2==1){									  // Negative Temperature
		Temp2 = (dataBuffer2[0] << 6) | (dataBuffer2[1] >> 2);
		Temp2&=0b01111111111111;
		Temp2^=0b01111111111111;
		temp2_value = (double)-Temp2/4;
	}
	else												  // Positive Temperature
	{
		Temp2 = (dataBuffer2[0] << 6) | (dataBuffer2[1] >> 2);
		temp2_value = ((double)Temp2 / 4);
	}

	if(!(dataBuffer2[1] & 0x01)){
		temp2_value = temp2_value*100-320; //room temp should've been 22.5, it was reading ambient 25.7
		SOAR_PRINT("Thermocouple 2 is reading %d.%d C \n\n", (int)temp2_value/100, (uint8_t)(int)temp2_value%100);
	}
}




