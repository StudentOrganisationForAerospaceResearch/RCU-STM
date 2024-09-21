/**
 ******************************************************************************
 * File Name          : TemperatureControl_Template.hpp
 * Description        : Functions defined, macros and etc for Temperature Control
 * 						Task
 ******************************************************************************
*/

#ifndef SOAR_TEMPERATURECONTROL_TEMPLATE_HPP_
#define SOAR_TEMPERATURECONTROL_TEMPLATE_HPP_

// Declaration of the template class
template <typename TEMP>
class TemperatureControl_Template {
public:
	TemperatureControl_Template(TEMP value);  // Constructor declaration

    void setValue(TEMP value);  // Setter method declaration
    TEMP getValue() const;      // Getter method declaration
    TEMP Run() const;

private:
    TEMP m_value;  // Private member to hold the value of type T
};


#endif  // TEMPLATECLASS_H
