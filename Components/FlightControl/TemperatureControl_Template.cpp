/*
 **********************************************************************************
 * File Name          : TemperatureControl_Template.cpp
 * Description        : This file is able to control the AC/Cooling unit by
 * 						reading the temperature from the thermocouples and when
 * 						it reaches a specified temperature that is too high, the
 * 						cooling unit is turned on, and turned off when the desired
 * 						temperature is reached.
 **********************************************************************************
*/

#ifndef TEMPLATECLASS_TPP
#define TEMPLATECLASS_TPP

#include <TemperatureControl_Template.hpp>

// GPIO initialize - Below code is NOT CORRECT
//typedef GPIO_OUTPUT_T<PF, 0> TRIGGER1;


/*
 * @brief Constructor for TemperatureControl
 */
template <typename TEMP>
TemperatureControl_Template<TEMP>::TemperatureControl_Template(TEMP value) : m_value(value) {}

/*
 * @brief Setter method definition
 */
template <typename TEMP>
void TemperatureControl_Template<TEMP>::setValue(TEMP value) {
    m_value = value;
}

/*
 * @brief Getter method definition
 */
template <typename TEMP>
TEMP TemperatureControl_Template<TEMP>::getValue() const {
    return m_value;
}

template <typename TEMP>
TEMP TemperatureControl_Template<TEMP>::Run() const {
    return m_value;
}

#endif  // TEMPERATURECONTROL_TPP

