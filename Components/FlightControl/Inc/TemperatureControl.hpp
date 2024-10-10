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

/* DEFINES */
#define ERROR_TEMPERATURE_VALUE 9999
#define TEMPERATURE_OFFSET -4.0 //in degrees Celsius
#define THERMOCOUPLE_SPI_TIMEOUT 100 //in ms

/* Macros/Enums ------------------------------------------------------------*/
//enum THERMOCOUPLE_TASK_COMMANDS {
//	THERMOCOUPLE_NULL = 0,
//	THERMOCOUPLE_REQUEST_NEW_SAMPLE,	// Get a new temperature sample
//	THERMOCOUPLE_REQUEST_TRANSMIT,		// Send the current temperature over the Protobuff
//	THERMOCOUPLE_REQUEST_DEBUG       	// Send the current temperature data over the Debug UART
//};

enum TARGET_CONTROLS{
	AC1 = 0,
	AC2,
	AC3
};

struct Temp_Control {
	TARGET_CONTROLS acUnit;
    int targetTemperature;
    bool isOn;
    int currTemperature;
};

class TemperatureControl : public Task
{
public:
    static TemperatureControl& Inst() {
        static TemperatureControl inst;
        return inst;
    }

    void InitTask();

protected:
    static void RunTask(void* pvParams) { TemperatureControl::Inst().Run(pvParams); }

    void Run(void* pvParams);
    void HandleCommand(Command& cm);
    void HandleRequestCommand(uint16_t taskCommand);

    void SampleThermocouple();
    int16_t ExtractTempurature(uint8_t temperatureData[]);

    //Fields
    uint8_t dataBuffer1[4] = {0};
    uint8_t dataBuffer2[4] = {0};
    int16_t temperature1 = 0;
    int16_t temperature2 = 0;

    Temp_Control tempControl[2];

private:
    TemperatureControl();
    TemperatureControl(const TemperatureControl&);
    TemperatureControl& operator=(const TemperatureControl&);
    bool acStatus;
};

#endif    // SOAR_TEMPERATURECONTROL_HPP_
