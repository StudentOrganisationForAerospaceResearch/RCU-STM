/**
 ******************************************************************************
 * File Name          : Main.hpp
 * Description        : Header file for Main.cpp, acts as an interface between
 *  STM32CubeIDE and our application.
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_MAIN_H
#define AVIONICS_INCLUDE_SOAR_MAIN_H
#include "Mutex.hpp"
#include "stm32l4xx_hal.h"

/* Interface Functions ------------------------------------------------------------------*/
/* These functions act as our program's 'main' and any functions inside CubeIDE's main --*/
void run_main();
void run_StartDefaultTask();

/* Global Functions ------------------------------------------------------------------*/
void print(const char* format, ...);
void soar_assert_debug(bool condition, const char* file, uint16_t line, const char* str = nullptr, ...);

/* Global Variable Interfaces ------------------------------------------------------------------*/
/* All must be extern from main_avionics.cpp -------------------------------------------------*/
namespace Global
{
	extern Mutex vaListMutex;
}


/* System Handles ------------------------------------------------------------------*/
//Timer Handles
extern TIM_HandleTypeDef htim2;      // ADC1

//ADC Handles
extern ADC_HandleTypeDef hadc1;      // ADC1
extern ADC_HandleTypeDef hadc2;      // ADC2

//I2C Handles

//SPI Handles
extern SPI_HandleTypeDef hspi2;      // SPI3 - Thermocouple Task

//CRC Handles
extern CRC_HandleTypeDef hcrc;       // CRC - Hardware CRC System Handle

//DMA Handles
extern DMA_HandleTypeDef hdma_uart4_rx; // DMA UART 4 RX -
extern DMA_HandleTypeDef hdma_uart5_rx; // DMA UART 5 RX -
extern DMA_HandleTypeDef hdma_uart5_tx; // DMA UART 5 TX -

namespace SystemHandles {
	// Aliases

	constexpr ADC_HandleTypeDef* ADC_CombustionChamber = &hadc1;
	constexpr ADC_HandleTypeDef* ADC_Battery = &hadc2;

	constexpr SPI_HandleTypeDef* SPI_Thermocouple1 = &hspi2;

	constexpr CRC_HandleTypeDef* CRC_Handle = &hcrc;

}

#endif /* AVIONICS_INCLUDE_SOAR_MAIN_H */
