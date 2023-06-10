

/**
 ******************************************************************************
 * File Name          : LoadCellTask.hpp
 * Description        : Primary LoadCell task, default task for the system.
 ******************************************************************************
*/
#ifndef SOAR_LOADCELLTASK_HPP_
#define SOAR_LOADCELLTASK_HPP_
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "hx711.h"


/* Macros/Enums ------------------------------------------------------------*/
enum LOADCELL_TASK_COMMANDS {
    LOADCELL_NONE = 0,
	LOADCELL_REQUEST_INIT,		  // Send the current load cell data during initializations over the Debug UARTs
	LOADCELL_REQUEST_TARE,		  // Send the current load cell data during tare over the Debug UART
	LOADCELL_REQUEST_CALIBRATE,   // Send the current load cell data during calibration over the Debug UART
    LOADCELL_REQUEST_NEW_SAMPLE,  // Get a new load cell sample, task will be blocked for polling time
    LOADCELL_REQUEST_TRANSMIT,    // Send the current load cell data over the Radio
    LOADCELL_REQUEST_DEBUG        // Send the current load cell data over the Debug UART
};


class LoadCellTask : public Task
{
public:
    static LoadCellTask& Inst() {
        static LoadCellTask inst;
        return inst;
    }

    void InitTask();
    int32_t GetNoLoad() {return loadcell.offset; }

protected:
    static void RunTask(void* pvParams) { LoadCellTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code


    void HandleCommand(Command& cm);
    void HandleRequestCommand(uint16_t taskCommand);

    void SampleLoadCellData();
    void LoadCellInit(GPIO_TypeDef *clk_gpio, uint16_t clk_pin, GPIO_TypeDef *dat_gpio, uint16_t dat_pin);
    void LoadCellTare();
    void LoadCellCalibrate();
    hx711_t loadcell;
    float knownmass = 16.5347; //This weight it in pounds and refers to the aluminum plate.
    int32_t value_noload;
    int32_t value_loadraw;
    float measuredWeight;


private:
    // Private Functions
    LoadCellTask();        // Private constructor
    LoadCellTask(const LoadCellTask&);                        // Prevent copy-construction
    LoadCellTask& operator=(const LoadCellTask&);            // Prevent assignment
};

#endif    // SOAR_LOADCELLTASK_HPP_