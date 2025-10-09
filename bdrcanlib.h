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
    */

// Top-level CanMessage so .cpp can define globals easily

#ifndef bdrcanlib_h
#define bdrcanlib_h
#include "Arduino.h"

struct CanMessage
{
    char *name;  // main name
    uint32_t id; // CAN ID
    char *alt;   // alternative name
    char *byte;
    int bit_start;
    int length;
    float min;
    float max;
    float scale;
    char *units;
    char *description;
};

struct messageStruct
{
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
};

class BDRCANLib
{
public:
    BDRCANLib(); // constructor

    // Convert a numeric string with comma decimals to float
    static float conv_to_dec(const String &s);

    messageStruct createMessage(uint32_t id, const uint8_t *data, uint8_t length);

    static float we_love_jaden_lee();

private:
    // Private members (if any)
};

// Global CAN message descriptors (defined in bdrcanlib.cpp)
static const CanMessage Set_AC_Current;
static const CanMessage Set_Brake_Current;
static const CanMessage Set_ERPM;
static const CanMessage Set_Position;
static const CanMessage Set_Relative_Current;
static const CanMessage Set_Relative_Brake_Current;
static const CanMessage Set_Digital_Output_1;
static const CanMessage Set_Digital_Output_2;
static const CanMessage Set_Digital_Output_3;
static const CanMessage Set_Digital_Output_4;
static const CanMessage Max_AC_Current;
static const CanMessage Set_Maximum_AC_Brake_Current;
static const CanMessage Max_DC_Current;
static const CanMessage Set_Maximum_DC_Brake_Current;
static const CanMessage Drive_Enable;

// Additional CAN messages
static const CanMessage erpm;
static const CanMessage duty_cycle;
static const CanMessage input_voltage;
static const CanMessage AC_current;
static const CanMessage DC_current;
static const CanMessage RESERVED_1;
static const CanMessage controller_temperature;
static const CanMessage motor_temperature;
static const CanMessage fault_code;
static const CanMessage RESERVED_2;
static const CanMessage Id;
static const CanMessage Iq;
static const CanMessage throttle_signal;
static const CanMessage brake_signal;
static const CanMessage digital_input_1;
static const CanMessage digital_input_2;
static const CanMessage digital_input_3;
static const CanMessage digital_input_4;
static const CanMessage digital_input_1_2;
static const CanMessage digital_input_2_2;
static const CanMessage digital_input_3_2;
static const CanMessage digital_input_4_2;
static const CanMessage drive_enable;
static const CanMessage capacitor_temp_limit;
static const CanMessage DC_current_limit;
static const CanMessage drive_enable_limit;
static const CanMessage igbt_acceleration_temperature_limit;
static const CanMessage igbt_temperature_limit;
static const CanMessage input_voltage_limit;
static const CanMessage motor_acceleration_temperature_limit;
static const CanMessage motor_temperature_limit;
static const CanMessage RPM_min_limit;
static const CanMessage RPM_max_limit;
static const CanMessage power_limit;
static const CanMessage reserved_3;
static const CanMessage reserved_4;
static const CanMessage CAN_map_version;

static const CanMessage ALL_MESSAGES[]; // Array of all messages
static const int NUM_MESSAGES;          // Number of messages in the array
static constexpr int MESSAGE_SIZE;         // default size of the CanMessage structure

#endif
