/**
  ******************************************************************************
  * File Name          : PressureTransducerTask.cpp
  *
  *    Source Info           : Based on Andromeda V3.31 Implementation
  *                         Andromeda_V3.31_Legacy/Core/Src/ReadBarometer.c
  *
  * Description        : This file contains constants and functions designed to
  *                      obtain accurate pressure and temperature readings from
  *                      the MS5607-02BA03 barometer on the flight board. A
  *                      thread task is included that will constantly loop,
  *                      reading and updating the passed BarometerData struct.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "PressureTransducerTask.hpp"
#include "main.h"
#include "Data.h"
#include "DebugTask.hpp"
#include "Task.hpp"
#include <time.h>


/* Macros --------------------------------------------------------------------*/

/* Structs -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief Default constructor, sets and sets up storage for member variables
 */
PressureTransducerTask::PressureTransducerTask() : Task(TASK_PRESSURE_TRANSDUCER_QUEUE_DEPTH_OBJS)
{
    data = (PressureTransducerData*)soar_malloc(sizeof(PressureTransducerData));
}

/**
 * @brief Creates a task for the FreeRTOS Scheduler
 */
void PressureTransducerTask::InitTask()
{
    // Make sure the task is not already initialized
    SOAR_ASSERT(rtTaskHandle == nullptr, "Cannot initialize PT task twice");

    // Start the task
    BaseType_t rtValue =
        xTaskCreate((TaskFunction_t)PressureTransducerTask::RunTask,
            (const char*)"PTTask",
            (uint16_t)TASK_PRESSURE_TRANSDUCER_STACK_DEPTH_WORDS,
            (void*)this,
            (UBaseType_t)TASK_PRESSURE_TRANSDUCER_PRIORITY,
            (TaskHandle_t*)&rtTaskHandle);

    //Ensure creation succeeded
    SOAR_ASSERT(rtValue == pdPASS, "PressureTransducerTask::InitTask() - xTaskCreate() failed");
}

/**
 * @brief PresssureTransducerTask run loop
 * @param pvParams Currently unused task context
 */
void PressureTransducerTask::Run(void * pvParams)
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
void PressureTransducerTask::HandleCommand(Command& cm)
{
    //TODO: Since this task will stall for a few milliseconds, we may need a way to eat the whole queue (combine similar eg. REQUEST commands and eat to WDG command etc)
    //TODO: Maybe a HandleEvtQueue instead that takes in the whole queue and eats the whole thing in order of non-blocking to blocking

    //Switch for the GLOBAL_COMMAND
    switch (cm.GetCommand()) {
    case REQUEST_COMMAND: {
        HandleRequestCommand(cm.GetTaskCommand());
    }
    case TASK_SPECIFIC_COMMAND: {
        break;
    }
    default:
        SOAR_PRINT("PressureTransducerTASK - Received Unsupported Command {%d}\n", cm.GetCommand());
        break;
    }

    //No matter what we happens, we must reset allocated data
    cm.Reset();
}

/**
 * @brief Handles a Request Command
 * @param taskCommand The command to handle
 */
void PressureTransducerTask::HandleRequestCommand(uint16_t taskCommand)
{
    //Switch for task specific command within DATA_COMMAND
    switch (taskCommand) {
    case PT_REQUEST_NEW_SAMPLE:
        SamplePressureTransducer();
        break;
    case PT_REQUEST_TRANSMIT:
        SOAR_PRINT("Stubbed: Pressure Transducer task transmit not implemented\n");
        break;
    case PT_REQUEST_DEBUG:
        //SOAR_PRINT("\t-- Pressure Transducer Data --\n");
        SOAR_PRINT("|PT_TASK| Pressure 1 (PSI): %d.%d, MCU Timestamp: %u\r\n", data->pressure_1 / 1000, data->pressure_1 % 1000,
        timestampPT);
        SOAR_PRINT("|PT_TASK| Pressure 2 (PSI): %d.%d, MCU Timestamp: %u\r\n", data->pressure_2 / 1000, data->pressure_2 % 1000,
                timestampPT);
        SOAR_PRINT("|PT_TASK| Pressure 3 (PSI): %d.%d, MCU Timestamp: %u\r\n", data->pressure_3 / 1000, data->pressure_3 % 1000,
                        timestampPT);
        SOAR_PRINT("|PT_TASK| Pressure 4 (PSI): %d.%d, MCU Timestamp: %u\r\n", data->pressure_4 / 1000, data->pressure_4 % 1000,
                        timestampPT);
        //SOAR_PRINT(" Pressure (psi): %d.%d\n", data->pressure_ / 1000, data->pressure_ % 1000);
        break;
    default:
        SOAR_PRINT("UARTTask - Received Unsupported REQUEST_COMMAND {%d}\n", taskCommand);
        break;
    }
}


void ADC_Select_CH1 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH2 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_2;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH3 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_3;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH4 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
/**
 * @brief This function reads and updates pressure readings
 *          from the pressure transducer.
 */
void PressureTransducerTask::SamplePressureTransducer()
{
	static const int PT_VOLTAGE_ADC_POLL_TIMEOUT = 5;
	static const double PRESSURE_SCALE = 1.5; // Value to scale to original voltage value
	uint32_t adcVal[4] = {};
	double pressureTransducerValue1 = 0;
	double pressureTransducerValue2 = 0;
	double pressureTransducerValue3 = 0;
	double pressureTransducerValue4 = 0;
	double vi = 0;

	//configure ADc channel
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);


	/* Functions -----------------------------------------------------------------*/
	ADC_Select_CH1();
	osDelay(1);
		HAL_ADC_Start(&hadc1);  // Enables ADC and starts conversion of regular channels
		if(HAL_ADC_PollForConversion(&hadc1, PT_VOLTAGE_ADC_POLL_TIMEOUT) == HAL_OK) { //Check if conversion is completed
			adcVal[0] = HAL_ADC_GetValue(&hadc1); // Get ADC Value
			HAL_ADC_Stop(&hadc1);
		}
	ADC_Select_CH2();
	osDelay(1);
		HAL_ADC_Start(&hadc1);  // Enables ADC and starts conversion of regular channels
		if(HAL_ADC_PollForConversion(&hadc1, PT_VOLTAGE_ADC_POLL_TIMEOUT) == HAL_OK) { //Check if conversion is completed
			adcVal[1] = HAL_ADC_GetValue(&hadc1); // Get ADC Value
			HAL_ADC_Stop(&hadc1);
		}
	ADC_Select_CH3();
	osDelay(1);
	HAL_ADC_Start(&hadc1);  // Enables ADC and starts conversion of regular channels
	if(HAL_ADC_PollForConversion(&hadc1, PT_VOLTAGE_ADC_POLL_TIMEOUT) == HAL_OK) { //Check if conversion is completed
		adcVal[2] = HAL_ADC_GetValue(&hadc1); // Get ADC Value
		HAL_ADC_Stop(&hadc1);
		}
	ADC_Select_CH4();
	osDelay(1);
		HAL_ADC_Start(&hadc1);  // Enables ADC and starts conversion of regular channels
		if(HAL_ADC_PollForConversion(&hadc1, PT_VOLTAGE_ADC_POLL_TIMEOUT) == HAL_OK) { //Check if conversion is completed
			adcVal[3] = HAL_ADC_GetValue(&hadc1); // Get ADC Value
			HAL_ADC_Stop(&hadc1);
		}
	vi = (adcVal[0]*3.3)/4095;

	SOAR_PRINT("here is the binary value of adcVal[0] %d\n", adcVal[0]);

	pressureTransducerValue1 = (250 * (vi * PRESSURE_SCALE) - 125) * 1000; // Multiply by 1000 to keep decimal places
	data->pressure_1 = (int32_t) pressureTransducerValue1; // Pressure in PSI

	vi = ((3.3/4095.0) * (adcVal[1])); // Converts 12 bit ADC value into voltage

	pressureTransducerValue2 = (250 * (vi * PRESSURE_SCALE) - 125) * 1000; // Multiply by 1000 to keep decimal places
	data->pressure_2 = (int32_t) pressureTransducerValue2; // Pressure in PSI


	vi = ((3.3/4095.0) * (adcVal[2])); // Converts 12 bit ADC value into voltage

	pressureTransducerValue3 = (250 * (vi * PRESSURE_SCALE) - 125) * 1000; // Multiply by 1000 to keep decimal places
	data->pressure_3 = (int32_t) pressureTransducerValue3; // Pressure in PSI

	vi = ((3.3/4095.0) * (adcVal[3])); // Converts 12 bit ADC value into voltage

	pressureTransducerValue4 = (250 * (vi * PRESSURE_SCALE) - 125) * 1000; // Multiply by 1000 to keep decimal places
	data->pressure_4 = (int32_t) pressureTransducerValue4; // Pressure in PSI



//	vi = ((3.3/4095) * (adcVal[1])); // Converts 12 bit ADC value into voltage
//		pressureTransducerValue2 = (250 * (vi * PRESSURE_SCALE) - 125) * 1000; // Multiply by 1000 to keep decimal places
//		data->pressure_2 = (int32_t) pressureTransducerValue2; // Pressure in PSI
//	vi = ((3.3/4095) * (adcVal[2])); // Converts 12 bit ADC value into voltage
//			pressureTransducerValue3 = (250 * (vi * PRESSURE_SCALE) - 125) * 1000; // Multiply by 1000 to keep decimal places
//			data->pressure_3 = (int32_t) pressureTransducerValue3; // Pressure in PSI
//	vi = ((3.3/4095) * (adcVal[3])); // Converts 12 bit ADC value into voltage
//			pressureTransducerValue4 = (250 * (vi * PRESSURE_SCALE) - 125) * 1000; // Multiply by 1000 to keep decimal places
//			data->pressure_4 = (int32_t) pressureTransducerValue4; // Pressure in PSI
	timestampPT = HAL_GetTick();
}
