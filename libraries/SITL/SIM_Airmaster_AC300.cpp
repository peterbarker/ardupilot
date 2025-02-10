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
 */
/*
  Simulator for Airmaster AC300 variable-pitch-prop controller
*/

#include "SIM_Airmaster_AC300.h"

#include <GCS_MAVLink/GCS.h>

#include <stdio.h>
#include <string.h>

using namespace SITL;

void Airmaster_AC300::consume_bytes(const uint8_t count)
{
    if (count > buflen) {
        AP_HAL::panic("Asked to consume more bytes than in buffer");
    }
    const uint8_t remaining = buflen - count;
    if (remaining != 0) {
        memmove(&buffer[count], &buffer[0], remaining);
    }
    buflen = remaining;
}

void Airmaster_AC300::update_read()
{
    const ssize_t n = read_from_autopilot(&buffer[buflen], ARRAY_SIZE(buffer) - buflen - 1);
    if (n <= 0) {
        return;
    }
    buflen += n;

    const char *cr = strchr(buffer, '\n');
    const uint8_t command_length = (cr-buffer+1);
    if (strict_parsing) {
        // ensure we have an entire line:
        if (cr == &buffer[0] || *(cr-1) != '\r') {
            AP_HAL::panic("Expected crlf");
        }
    }

    if (cr == nullptr) {
        if (strict_parsing) {
            AP_HAL::panic("Didn't find a line");
        }
        if (buflen == ARRAY_SIZE(buffer) - 1) {
            // nuke it all
            memset(buffer, '\0', ARRAY_SIZE(buffer));
            buflen = 0;
        }
        return;
    }

    // here we should have a complete line in the buffer.
    const char *command_AT = "AT\r\n";
    if (strncmp(buffer, command_AT, MIN(command_length, strlen(command_AT))) == 0) {
        const char *response = "OK\n";
        write_to_autopilot(response, strlen(response));
        consume_bytes(command_length);
        return;
    }

    if (strncmp(buffer, "RC_S=", ARRAY_SIZE(buffer))) {
        uint32_t rpm;
        if (sscanf(&buffer[5], "%u", &rpm) == 1) {
            desired_rpm = rpm;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "desired_rpm=%u", desired_rpm);
            consume_bytes(command_length);
            return;
        }
    }

    if (strict_parsing) {
        AP_HAL::panic("Unrecognised line (%s)", buffer);
    }
}

void Airmaster_AC300::update()
{
    update_read();
}
