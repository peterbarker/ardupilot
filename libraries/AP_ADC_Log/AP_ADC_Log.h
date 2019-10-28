/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.


   Author: Francisco Ferreira

 */
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>

class AP_ADC_Log {

public:
    AP_ADC_Log() { AP_Param::setup_object_defaults(this, var_info); };
    
    /* Do not allow copies */
    AP_ADC_Log(const AP_ADC_Log &other) = delete;
    AP_ADC_Log &operator=(const AP_ADC_Log&) = delete;

    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:
    static const uint8_t NUM_ADCs = 2;

    AP_Int8 _input[NUM_ADCs];
    AP_HAL::AnalogSource* _adc_source[NUM_ADCs];
};
