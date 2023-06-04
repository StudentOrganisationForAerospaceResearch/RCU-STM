/**
 ******************************************************************************
 * File Name          : GPIO.hpp
 * Description        :
 *
 *	GPIO contains all GPIO pins wrapped in a namespace and corresponding functions
 *
 *	All GPIO pins should be controlled through this abstraction layer to ensure readable control.
 *
 ******************************************************************************
*/
#ifndef AVIONICS_INCLUDE_SOAR_CORE_GPIO_H
#define AVIONICS_INCLUDE_SOAR_CORE_GPIO_H
#include "SystemDefines.hpp"
#include "main.h"
#include "stm32l4xx_hal.h"

#define RELAY_OPEN GPIO_PIN_SET
#define RELAY_CLOSE GPIO_PIN_RESET

namespace GPIO
{
	namespace LED1
	{
		inline void On() { HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET); }
		inline void Off() { HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET); }
		inline void Toggle() { HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin); }

		inline bool IsOn() { return HAL_GPIO_ReadPin(LED_1_GPIO_Port, LED_1_Pin) == GPIO_PIN_SET; }
	}

	namespace LED2
	{
		inline void On() { HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET); }
		inline void Off() { HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET); }
		inline void Toggle() { HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin); }

		inline bool IsOn() { return HAL_GPIO_ReadPin(LED_1_GPIO_Port, LED_1_Pin) == GPIO_PIN_SET; }
	}

	namespace AC1
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace AC2
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace PBV1
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace PBV2
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace PBV3
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace SOL1
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace SOL2
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace SOL3
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace SOL4
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace SOL5
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace SOL6
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace SOL7
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace SOL8A
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}

	namespace SOL8B
	{
		inline void Open() { HAL_GPIO_WritePin(, , RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(, , RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(, ); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(, ) == RELAY_CLOSE; }
	}
	
}

#endif /* AVIONICS_INCLUDE_SOAR_CORE_GPIO_H */
