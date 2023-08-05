/**
 ******************************************************************************
 * File Name          : LoadCellTask.hpp
 * Description        : Primary LoadCell task, default task for the system.
 ******************************************************************************
*/
#ifndef SOAR_LOADCELLTASK_HPP_
#define SOAR_LOADCELLTASK_HPP_

#define LBS_TO_GRAMS(lbs) ((lbs) * 453.592)
#define GRAMS_TO_LBS(grams) ((grams) * 0.00220462)
#include "Task.hpp"
#include "SystemDefines.hpp"
#include "hx711.h"


/* Macros/Enums ------------------------------------------------------------*/
enum LOADCELL_TASK_COMMANDS {
    LOADCELL_NONE = 0,
	NOS1_LOADCELL_REQUEST_TARE,		  			// Tare NOS1 load cell
	NOS1_LOADCELL_REQUEST_CALIBRATE,   			// Calibrate NOS1 load cell with known mass
	NOS2_LOADCELL_REQUEST_TARE,		  			// Tare NOS2 load cell
	NOS2_LOADCELL_REQUEST_CALIBRATE,   			// Calibrate NOS2 load cell with known mass
    LOADCELL_REQUEST_NEW_SAMPLE,  				// Sample both NOS1 and NOS2 load cells, task will be blocked for polling time
    LOADCELL_REQUEST_TRANSMIT,    		 		// Send the current load cell data over the Radio
	LOADCELL_REQUEST_CALIBRATION_DEBUG, 		// Print the offset, scale, and known mass used for calibration
    LOADCELL_REQUEST_DEBUG        				// Send the current load cell data over the Debug UART
};

struct TwoFillLoadCellSample
{
	float nos1_adc_value;
	float nos2_adc_value;
	uint32_t timestamp_ms;
};

class LoadCellTask : public Task
{
public:
    static LoadCellTask& Inst() {
        static LoadCellTask inst;
        return inst;
    }

    void InitTask();
    void SetCalibrationMassGrams(const float mass_g) { calibration_mass_g = mass_g; };
    const float getCalibrationMassGrams() { return calibration_mass_g; };

protected:
    static void RunTask(void* pvParams) { LoadCellTask::Inst().Run(pvParams); } // Static Task Interface, passes control to the instance Run();

    void Run(void * pvParams); // Main run code

    void HandleCommand(Command& cm);
    void HandleRequestCommand(uint16_t taskCommand);

    void SampleLoadCellData();
    void LoadCellTare(hx711_t* loadcell);
    void LoadCellCalibrate(hx711_t* loadcell);
    void TransmitProtocolLoadCellData();

    hx711_t nos1_loadcell;
    hx711_t nos2_loadcell;
    TwoFillLoadCellSample two_fill_mass_sample;
    float calibration_mass_g;								 // last used calibration mass

private:
    // Private Functions
    LoadCellTask();        									  // Private constructor
    LoadCellTask(const LoadCellTask&);                        // Prevent copy-construction
    LoadCellTask& operator=(const LoadCellTask&);             // Prevent assignment
};

#endif    // SOAR_LOADCELLTASK_HPP_
