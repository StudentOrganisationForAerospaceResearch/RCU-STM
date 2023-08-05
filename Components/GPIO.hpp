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

#define SOL4_OPEN GPIO_PIN_RESET
#define SOL4_CLOSE GPIO_PIN_SET

#define PADBOX_IGNITE RELAY_CLOSE
#define PADBOX_KILL RELAY_OPEN

#define IS_CONTINUOUS GPIO_PIN_SET

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

	namespace SHEDAC
	{
		inline void Open() { HAL_GPIO_WritePin(SHEDAC_GPIO_Port, SHEDAC_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SHEDAC_GPIO_Port, SHEDAC_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SHEDAC_GPIO_Port, SHEDAC_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(SHEDAC_GPIO_Port, SHEDAC_Pin) == RELAY_OPEN; }
	}

	namespace PADBOX1
	{
		inline void Ignite() { HAL_GPIO_WritePin(PADBOX1_GPIO_Port, PADBOX1_Pin, PADBOX_IGNITE); }
        inline void Kill() { HAL_GPIO_WritePin(PADBOX1_GPIO_Port, PADBOX1_Pin, PADBOX_KILL); }
	    inline void Toggle() { HAL_GPIO_TogglePin(PADBOX1_GPIO_Port, PADBOX1_Pin); }

		inline bool IsLive() { return HAL_GPIO_ReadPin(PADBOX1_GPIO_Port, PADBOX1_Pin) == PADBOX_IGNITE; }
	}

	namespace PADBOX2
	{
		inline void Ignite() { HAL_GPIO_WritePin(PADBOX2_GPIO_Port, PADBOX2_Pin, PADBOX_IGNITE); }
        inline void Kill() { HAL_GPIO_WritePin(PADBOX2_GPIO_Port, PADBOX2_Pin, PADBOX_KILL); }
        inline void Toggle() { HAL_GPIO_TogglePin(PADBOX2_GPIO_Port, PADBOX2_Pin); }

		inline bool IsLive() { return HAL_GPIO_ReadPin(PADBOX2_GPIO_Port, PADBOX2_Pin) == PADBOX_IGNITE; }
	}

	namespace PBV1
	{
		inline void Open() { HAL_GPIO_WritePin(PBV1_GPIO_Port, PBV1_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(PBV1_GPIO_Port, PBV1_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(PBV1_GPIO_Port, PBV1_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(PBV1_GPIO_Port, PBV1_Pin) == RELAY_OPEN; }
	}

	namespace PBV2
	{
		inline void Open() { HAL_GPIO_WritePin(PBV2_GPIO_Port, PBV2_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(PBV2_GPIO_Port, PBV2_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(PBV2_GPIO_Port, PBV2_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(PBV2_GPIO_Port, PBV2_Pin) == RELAY_OPEN; }
	}

	namespace PBV3
	{
		inline void Open() { HAL_GPIO_WritePin(PBV3_GPIO_Port, PBV3_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(PBV3_GPIO_Port, PBV3_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(PBV3_GPIO_Port, PBV3_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(PBV3_GPIO_Port, PBV3_Pin) == RELAY_OPEN; }
	}

	namespace SOL4
	{
		inline void Open() { HAL_GPIO_WritePin(SOL4_GPIO_Port, SOL4_Pin, SOL4_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL4_GPIO_Port, SOL4_Pin, SOL4_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL4_GPIO_Port, SOL4_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(SOL4_GPIO_Port, SOL4_Pin) == SOL4_OPEN; }
	}

	namespace SOL5
	{
		inline void Open() { HAL_GPIO_WritePin(SOL5_GPIO_Port, SOL5_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL5_GPIO_Port, SOL5_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL5_GPIO_Port, SOL5_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(SOL5_GPIO_Port, SOL5_Pin) == RELAY_OPEN; }
	}

	namespace SOL6
	{
		inline void Open() { HAL_GPIO_WritePin(SOL6_GPIO_Port, SOL6_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL6_GPIO_Port, SOL6_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL6_GPIO_Port, SOL6_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(SOL6_GPIO_Port, SOL6_Pin) == RELAY_OPEN; }
	}

	namespace SOL7
	{
		inline void Open() { HAL_GPIO_WritePin(SOL7_GPIO_Port, SOL7_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL7_GPIO_Port, SOL7_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL7_GPIO_Port, SOL7_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(SOL7_GPIO_Port, SOL7_Pin) == RELAY_OPEN; }
	}

	namespace SOL8A
	{
		inline void Open() { HAL_GPIO_WritePin(SOL8A_GPIO_Port, SOL8A_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL8A_GPIO_Port, SOL8A_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL8A_GPIO_Port, SOL8A_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(SOL8A_GPIO_Port, SOL8A_Pin) == RELAY_OPEN; }
	}

	namespace SOL8B
	{
		inline void Open() { HAL_GPIO_WritePin(SOL8B_GPIO_Port, SOL8B_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL8B_GPIO_Port, SOL8B_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL8B_GPIO_Port, SOL8B_Pin); }

		inline bool IsOpen() { return HAL_GPIO_ReadPin(SOL8B_GPIO_Port, SOL8B_Pin) == RELAY_OPEN; }
	}

	namespace CONT_CK0
	{
		inline bool IsContinuous() { return HAL_GPIO_ReadPin(CONT_CK0_GPIO_Port, CONT_CK0_Pin) == IS_CONTINUOUS; }
	}

	namespace CONT_CK1
	{
		inline bool IsContinuous() { return HAL_GPIO_ReadPin(CONT_CK1_GPIO_Port, CONT_CK1_Pin) == IS_CONTINUOUS; }
	}
	
}

#endif /* AVIONICS_INCLUDE_SOAR_CORE_GPIO_H */
