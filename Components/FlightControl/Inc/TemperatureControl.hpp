/**
 ******************************************************************************
 * File Name          : TemperatureControl.hpp
 * Description        : Functions defined, macros and etc for Temperature Control
 * 						Task
 ******************************************************************************
*/
#ifndef SOAR_TEMPERATURECONTROL_HPP_
#define SOAR_TEMPERATURECONTROL_HPP_

/* INCLUDES */
#include "Task.hpp"
#include "SystemDefines.hpp"

// Macros/Enums

enum TEMPERATURE_TASK_COMMANDS{
	TEMPERATURE_TASK_RTOS_PRIORITY,
	TEMPERATURE_TASK_QUEUE_DEPTH_OBJS,
	TEMPERATURE_TASK_STACK_DEPTH_WORDS
};

#define ERROR_TEMPERATURE_VALUE 9999
#define TEMPERATURE_OFFSET -4.0 //in degrees Celsius
#define THERMOCOUPLE_SPI_TIMEOUT 100 //in ms

class TemperatureControl : public Task
{
public:
    static TemperatureControl& Inst() {
        static TemperatureControl inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { TemperatureControl::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code
    void HandleCommand(Command& cm);
    void HandleRequestCommand(uint16_t taskCommand);

    void SampleThermocouple();
    int16_t ExtractTempurature(uint8_t temperatureData[]);

    //Fields
        uint8_t dataBuffer1[4] = {0};
        uint8_t dataBuffer2[4] = {0};
        int16_t temperature1 = 0;
        int16_t temperature2 = 0;

private:
    // Private Functions
    TemperatureControl();        // Private constructor
    TemperatureControl(const TemperatureControl&);                        // Prevent copy-construction
    TemperatureControl& operator=(const TemperatureControl&);            // Prevent assignment
    bool AcStatus;
};

#endif    // SOAR_TEMPERATURECONTROL_HPP_
