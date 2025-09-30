/*
    bdrcan.h - Library for bulldog racing CAN id protocol, as defined by 
    https://www.drivetraininnovation.com/file-share/62d657cb-4878-478e-a06c-e22040fd226f (can manual v2.4)
    Created by Jaden Lee, Sept 29th 2025, Yale class of 2029.
    Use with credit.

    Copyright 2025 Jaden Lee

    Permission is hereby granted, free of charge, 
    to any person obtaining a copy of this software and associated documentation files (the “Software”), 
    to deal in the Software without restriction, including without limitation the rights to use, copy, modify, 
    merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the 
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
    DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    (MIT License)
*/

#ifndef BRDCANLIB_H
#define BRDCANLIB_H

#include <Arduino.h>  // always include Arduino core
#include <FlexCAN_T4.h>  // include the FlexCAN library

class BDRCANLib {
public:
    BDRCANLib();   // constructor
    void begin();         // initialization

    struct CanMessage {
        uint32_t id;       // CAN ID
        String notes;      // alternate name
        int byte;
        int bit_start;
        int length;
        float min;
        float max;
        float scale;
        String units;
        String description;
        
    };

    struct messageStruct {
        uint32_t id;
        uint8_t data[8];
        uint8_t length;
    };

    messageStruct createMessage(uint32_t id, const uint8_t* data, uint8_t length);

private:
// CAN messages
    CanMessage Set_AC_Current;
    CanMessage Set_Brake_Current;
    CanMessage Set_ERPM;
    CanMessage Set_Position;
    CanMessage Set_Relative_Current;
    CanMessage Set_Relative_Brake_Current;
    CanMessage Set_Digital_Output_1;
    CanMessage Set_Digital_Output_2;
    CanMessage Set_Digital_Output_3;
    CanMessage Set_Digital_Output_4;
    CanMessage Max_AC_Current;
    CanMessage Set_Maximum_AC_Brake_Current;
    CanMessage Max_DC_Current;
    CanMessage Set_Maximum_DC_Brake_Current;
    CanMessage Drive_Enable;

    // Additional CAN messages
    CanMessage erpm;
    CanMessage duty_cycle;
    CanMessage input_voltage;
    CanMessage AC_current;
    CanMessage DC_current;
    CanMessage RESERVED_1;
    CanMessage controller_temperature;
    CanMessage motor_temperature;
    CanMessage fault_code;
    CanMessage RESERVED_2;
    CanMessage Id;
    CanMessage Iq;
    CanMessage throttle_signal;
    CanMessage brake_signal;
    CanMessage digital_input_1;
    CanMessage digital_input_2;
    CanMessage digital_input_3;
    CanMessage digital_input_4;
    CanMessage digital_input_1_2;
    CanMessage digital_input_2_2;
    CanMessage digital_input_3_2;
    CanMessage digital_input_4_2;
    CanMessage drive_enable;
    CanMessage capacitor_temp_limit;
    CanMessage DC_current_limit;
    CanMessage drive_enable_limit;
    CanMessage igbt_acceleration_temperature_limit;
    CanMessage igbt_temperature_limit;
    CanMessage input_voltage_limit;
    CanMessage motor_acceleration_temperature_limit;
    CanMessage motor_temperature_limit;
    CanMessage RPM_min_limit;
    CanMessage RPM_max_limit;
    CanMessage power_limit;
    CanMessage reserved_3;
    CanMessage reserved_4;
    CanMessage CAN_map_version;
};

#endif
