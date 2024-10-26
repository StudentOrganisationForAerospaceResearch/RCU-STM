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
enum THERMOCOUPLE_TASK_COMMANDS {
	THERMOCOUPLE_NULL = 0,
	SET_TARGET_TEMP,
	SET_TARGET_STATE,
	SET_CURRENT_TEMP
};

enum class TARGET_CONTROLS{
	AC1 = 0,
	AC2,
	AC3,
	NUMBER_OF_CONTROLS, // NOTE: Always keep this as the last item
};

//diff functions for each in the struct and and to be able to store them
struct Temp_Control {
	TARGET_CONTROLS acUnit;
    uint8_t targetTemperature;
    bool isOn;
    uint8_t currTemperature; //will be updated as temp being passed through from GUI
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

private:
    TemperatureControl();
    TemperatureControl(const TemperatureControl&);
    TemperatureControl& operator=(const TemperatureControl&);
    bool acStatus;
};

#endif    // SOAR_TEMPERATURECONTROL_HPP_
