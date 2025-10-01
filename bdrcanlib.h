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
    struct CanMessage {
        String name;      // main name
        uint32_t id;       // CAN ID
        String alt;     // alternative name
        String byte;
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

    class BDRCANLib {
    public:
        BDRCANLib();   // constructor

        // Convert a numeric string with comma decimals to float
        static float conv_to_dec(const String& s);

        messageStruct createMessage(uint32_t id, const uint8_t* data, uint8_t length);
    

        // Global CAN message descriptors (defined in bdrcanlib.cpp)
        extern CanMessage Set_AC_Current;
        extern CanMessage Set_Brake_Current;
        extern CanMessage Set_ERPM;
        extern CanMessage Set_Position;
        extern CanMessage Set_Relative_Current;
        extern CanMessage Set_Relative_Brake_Current;
        extern CanMessage Set_Digital_Output_1;
        extern CanMessage Set_Digital_Output_2;
        extern CanMessage Set_Digital_Output_3;
        extern CanMessage Set_Digital_Output_4;
        extern CanMessage Max_AC_Current;
        extern CanMessage Set_Maximum_AC_Brake_Current;
        extern CanMessage Max_DC_Current;
        extern CanMessage Set_Maximum_DC_Brake_Current;
        extern CanMessage Drive_Enable;

        // Additional CAN messages
        extern CanMessage erpm;
        extern CanMessage duty_cycle;
        extern CanMessage input_voltage;
        extern CanMessage AC_current;
        extern CanMessage DC_current;
        extern CanMessage RESERVED_1;
        extern CanMessage controller_temperature;
        extern CanMessage motor_temperature;
        extern CanMessage fault_code;
        extern CanMessage RESERVED_2;
        extern CanMessage Id;
        extern CanMessage Iq;
        extern CanMessage throttle_signal;
        extern CanMessage brake_signal;
        extern CanMessage digital_input_1;
        extern CanMessage digital_input_2;
        extern CanMessage digital_input_3;
        extern CanMessage digital_input_4;
        extern CanMessage digital_input_1_2;
        extern CanMessage digital_input_2_2;
        extern CanMessage digital_input_3_2;
        extern CanMessage digital_input_4_2;
        extern CanMessage drive_enable;
        extern CanMessage capacitor_temp_limit;
        extern CanMessage DC_current_limit;
        extern CanMessage drive_enable_limit;
        extern CanMessage igbt_acceleration_temperature_limit;
        extern CanMessage igbt_temperature_limit;
        extern CanMessage input_voltage_limit;
        extern CanMessage motor_acceleration_temperature_limit;
        extern CanMessage motor_temperature_limit;
        extern CanMessage RPM_min_limit;
        extern CanMessage RPM_max_limit;
        extern CanMessage power_limit;
        extern CanMessage reserved_3;
        extern CanMessage reserved_4;
        extern CanMessage CAN_map_version;

        static float we_love_jaden_lee();
    private:
        // Private members (if any)
    };

#endif
