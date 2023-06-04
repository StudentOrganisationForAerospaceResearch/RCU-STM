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

#define IS_CONTINUOUS GPIO_PIN_SET

#define AC1_GPIO_Port RELAY0_GPIO_Port
#define AC1_Pin RELAY0_Pin
#define AC2_GPIO_Port RELAY1_GPIO_Port
#define AC2_Pin RELAY1_Pin
#define AC3_GPIO_Port RELAY2_GPIO_Port
#define AC3_Pin RELAY2_Pin

#define PBV1_GPIO_Port RELAY3_GPIO_Port
#define PBV1_Pin RELAY3_Pin
#define PBV2_GPIO_Port RELAY4_GPIO_Port
#define PBV2_Pin RELAY4_Pin
#define PBV3_GPIO_Port RELAY5_GPIO_Port
#define PBV3_Pin RELAY5_Pin

#define SOL1_GPIO_Port RELAY3_GPIO_Port
#define SOL1_Pin RELAY3_Pin
#define SOL2_GPIO_Port RELAY4_GPIO_Port
#define SOL2_Pin RELAY4_Pin
#define SOL3_GPIO_Port RELAY5_GPIO_Port
#define SOL3_Pin RELAY5_Pin

#define SOL4_GPIO_Port RELAY6_GPIO_Port
#define SOL4_Pin RELAY6_Pin
#define SOL5_GPIO_Port RELAY7_GPIO_Port
#define SOL5_Pin RELAY7_Pin
#define SOL6_GPIO_Port RELAY8_GPIO_Port
#define SOL6_Pin RELAY8_Pin
#define SOL7_GPIO_Port RELAY9_GPIO_Port
#define SOL7_Pin RELAY9_Pin
#define SOL8A_GPIO_Port RELAY10_GPIO_Port
#define SOL8A_Pin RELAY10_Pin
#define SOL8B_GPIO_Port RELAY11_GPIO_Port
#define SOL8B_Pin RELAY11_Pin

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
		inline void Open() { HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(AC1_GPIO_Port, AC1_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(AC1_GPIO_Port, AC1_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(AC1_GPIO_Port, AC1_Pin) == RELAY_CLOSE; }
	}

	namespace AC2
	{
		inline void Open() { HAL_GPIO_WritePin(AC2_GPIO_Port, AC2_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(AC2_GPIO_Port, AC2_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(AC2_GPIO_Port, AC2_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(AC2_GPIO_Port, AC2_Pin) == RELAY_CLOSE; }
	}

	namespace AC3
	{
		inline void Open() { HAL_GPIO_WritePin(AC3_GPIO_Port, AC3_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(AC3_GPIO_Port, AC3_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(AC3_GPIO_Port, AC3_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(AC3_GPIO_Port, AC3_Pin) == RELAY_CLOSE; }
	}

	namespace PBV1
	{
		inline void Open() { HAL_GPIO_WritePin(PBV1_GPIO_Port, PBV1_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(PBV1_GPIO_Port, PBV1_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(PBV1_GPIO_Port, PBV1_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(PBV1_GPIO_Port, PBV1_Pin) == RELAY_CLOSE; }
	}

	namespace PBV2
	{
		inline void Open() { HAL_GPIO_WritePin(PBV2_GPIO_Port, PBV2_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(PBV2_GPIO_Port, PBV2_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(PBV2_GPIO_Port, PBV2_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(PBV2_GPIO_Port, PBV2_Pin) == RELAY_CLOSE; }
	}

	namespace PBV3
	{
		inline void Open() { HAL_GPIO_WritePin(PBV3_GPIO_Port, PBV3_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(PBV3_GPIO_Port, PBV3_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(PBV3_GPIO_Port, PBV3_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(PBV3_GPIO_Port, PBV3_Pin) == RELAY_CLOSE; }
	}

	namespace SOL1
	{
		inline void Open() { HAL_GPIO_WritePin(SOL1_GPIO_Port, SOL1_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL1_GPIO_Port, SOL1_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL1_GPIO_Port, SOL1_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(SOL1_GPIO_Port, SOL1_Pin) == RELAY_CLOSE; }
	}

	namespace SOL2
	{
		inline void Open() { HAL_GPIO_WritePin(SOL2_GPIO_Port, SOL2_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL2_GPIO_Port, SOL2_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL2_GPIO_Port, SOL2_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(SOL2_GPIO_Port, SOL2_Pin) == RELAY_CLOSE; }
	}

	namespace SOL3
	{
		inline void Open() { HAL_GPIO_WritePin(SOL3_GPIO_Port, SOL3_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL3_GPIO_Port, SOL3_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL3_GPIO_Port, SOL3_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(SOL3_GPIO_Port, SOL3_Pin) == RELAY_CLOSE; }
	}

	namespace SOL4
	{
		inline void Open() { HAL_GPIO_WritePin(SOL4_GPIO_Port, SOL4_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL4_GPIO_Port, SOL4_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL4_GPIO_Port, SOL4_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(SOL4_GPIO_Port, SOL4_Pin) == RELAY_CLOSE; }
	}

	namespace SOL5
	{
		inline void Open() { HAL_GPIO_WritePin(SOL5_GPIO_Port, SOL5_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL5_GPIO_Port, SOL5_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL5_GPIO_Port, SOL5_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(SOL5_GPIO_Port, SOL5_Pin) == RELAY_CLOSE; }
	}

	namespace SOL6
	{
		inline void Open() { HAL_GPIO_WritePin(SOL6_GPIO_Port, SOL6_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL6_GPIO_Port, SOL6_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL6_GPIO_Port, SOL6_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(SOL6_GPIO_Port, SOL6_Pin) == RELAY_CLOSE; }
	}

	namespace SOL7
	{
		inline void Open() { HAL_GPIO_WritePin(SOL7_GPIO_Port, SOL7_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL7_GPIO_Port, SOL7_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL7_GPIO_Port, SOL7_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(SOL7_GPIO_Port, SOL7_Pin) == RELAY_CLOSE; }
	}

	namespace SOL8A
	{
		inline void Open() { HAL_GPIO_WritePin(SOL8A_GPIO_Port, SOL8A_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL8A_GPIO_Port, SOL8A_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL8A_GPIO_Port, SOL8A_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(SOL8A_GPIO_Port, SOL8A_Pin) == RELAY_CLOSE; }
	}

	namespace SOL8B
	{
		inline void Open() { HAL_GPIO_WritePin(SOL8B_GPIO_Port, SOL8B_Pin, RELAY_OPEN); }
		inline void Close() { HAL_GPIO_WritePin(SOL8B_GPIO_Port, SOL8B_Pin, RELAY_CLOSE); }
		inline void Toggle() { HAL_GPIO_TogglePin(SOL8B_GPIO_Port, SOL8B_Pin); }

		inline bool IsClosed() { return HAL_GPIO_ReadPin(SOL8B_GPIO_Port, SOL8B_Pin) == RELAY_CLOSE; }
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
