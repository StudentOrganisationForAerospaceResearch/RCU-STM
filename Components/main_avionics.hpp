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
/* This should be the only place externs are allowed -------------------------------*/
//UART Handles
extern UART_HandleTypeDef huart1;   // UART1 - SOB
extern UART_HandleTypeDef huart2;   // UART2 - UMBILICAL
extern UART_HandleTypeDef huart3;   // UART3 - RADIO
extern UART_HandleTypeDef hlpuart1; // LPUART1 - PI

//timer Handles
extern TIM_HandleTypeDef htim2;      // ADC1

//ADC Handles
extern ADC_HandleTypeDef hadc1;      // ADC1
extern ADC_HandleTypeDef hadc2;      // ADC2
//I2C Handles
//extern I2C_HandleTypeDef hi2c1;      // I2C1 -- EEPROM (? - Do we still have an I2C EEPROM)

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
	constexpr UART_HandleTypeDef* UART_SOB = &huart1;
	constexpr UART_HandleTypeDef* UART_Umbilical = &huart2;
	constexpr UART_HandleTypeDef* UART_Radio = &huart3;
	constexpr UART_HandleTypeDef* UART_Debug = &huart2;
	constexpr UART_HandleTypeDef* UART_PI = &hlpuart1;

	constexpr ADC_HandleTypeDef* ADC_CombustionChamber = &hadc1;
	constexpr ADC_HandleTypeDef* ADC_Battery = &hadc2;

	constexpr SPI_HandleTypeDef* SPI_Thermocouple1 = &hspi2;

	constexpr CRC_HandleTypeDef* CRC_Handle = &hcrc;

	// DMA Aliases

}

#endif /* AVIONICS_INCLUDE_SOAR_MAIN_H */
