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

#include <AP_Logger/AP_Logger.h>


#include "AP_ADC_Log.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_ADC_Log::var_info[] = {
    // @Param: 1
    // @DisplayName: ADC Log Input 1
    // @Description: This controls which input is logged
    // @Values: -1:Disabled, 2:Pixhawk2_PM1_Volt, 3:Pixhawk2_PM1_Curr, 13:Pixhawk2_PM2_Volt, 14:Pixhawk2_PM2_Curr, 15:Pixhawk2 ADC
    // @User: Standard
    AP_GROUPINFO("1",  1, AP_ADC_Log, _input[0], -1),

    // @Param: 2
    // @DisplayName: ADC Log Input 2
    // @Description: This controls which input is logged
    // @Values: -1:Disabled, 2:Pixhawk2_PM1_Volt, 3:Pixhawk2_PM1_Curr, 13:Pixhawk2_PM2_Volt, 14:Pixhawk2_PM2_Curr, 15:Pixhawk2 ADC
    // @User: Standard
    AP_GROUPINFO("2",  2, AP_ADC_Log, _input[1], -1),

    AP_GROUPEND
};

void AP_ADC_Log::update()
{
    float adc_values[NUM_ADCs] = { 0.0f };

    for (uint8_t i = 0; i < NUM_ADCs; i++) {
        if (!_adc_source[i] && _input[i] != -1) {
            _adc_source[i] = hal.analogin->channel(_input[i]);
        }
        if (_adc_source[i]) {
            _adc_source[i]->set_pin(_input[i]);
            adc_values[i] = _adc_source[i]->voltage_average();
        }
    }

    AP_Logger& log = AP::logger();

    log.Write("ADCL", "TimeUS,ADC1,ADC2", "svv", "F00", "Qff", AP_HAL::micros64(), adc_values[0], adc_values[1]);
}
