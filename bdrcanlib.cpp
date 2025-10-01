#include "Arduino.h"
#include "bdrcanlib.h"

BDRCANLib::BDRCANLib() {
    // constructor body left intentionally empty — real init can go in begin()
}
messageStruct BDRCANLib::createMessage(uint32_t id, const uint8_t* data, uint8_t length) {
    messageStruct m;
    m.id = id;
    m.length = length;
    if (data && length > 0) {
        memcpy(m.data, data, length);
    } else {
        memset(m.data, 0, sizeof(m.data));
    }
    return m;
}

float BDRCANLib::conv_to_dec(const String& s) {
    String tmp = s;
    tmp.replace(',', '.');
    return tmp.toFloat();
}

/*
 * Define every CAN ID used in the system.
 * Add or modify as needed for your application.
 */
const CanMessage Set_AC_Current = {
    "Set AC Current",
    0x01,
    "ac current",
    "0",
    1,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "A_pk",
    "This command sets the target motor AC current (peak, not RMS). When the controller receives this message, it automatically switches to current control mode. This value must not be above the limits of the inverter and must be multiplied by 10 before sending. This is a signed parameter, and the sign represents the direction of the torque which correlates with the motor AC current. (For the correlation, please refer to the motor parameters)"
};

const CanMessage Set_Brake_Current = {
    "Set Brake current",
    0x02,
    "target brake current",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "A_pk",
    "Targets the brake current of the motor. It will result negative torque relatively to the forward direction of the motor. This value must be multiplied by 10 before sending, only positive currents are accepted."
};

const CanMessage Set_ERPM = {
    "Set ERPM",
    0x03,
    "Set speed (ERPM)",
    "0-3",
    0,
    8,
    -2147483648.0f,
    2147483647.0f,
    1.0f,
    "ERPM",
    "This command enables the speed control of the motor with a target ERPM. This is a signed parameter, and the sign represents the direction of the spinning. For better operation you need to tune the PID of speed control. Equation: ERPM = Motor RPM * number of the motor pole pairs."
};

const CanMessage Set_Position = {
    "Set Position",
    0x04,
    "Target position",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "degree",
    "This value targets the desired position of the motor in degrees. This command is used to hold a position of the motor. This feature is enabled only if encoder is used as position sensor. The value has to be multiplied by 10 before sending."
};

const CanMessage Set_Relative_Current = {
    "Set Relative current",
    0x05,
    "Set relative current",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "%",
    "This command sets a relative AC current to the minimum and maximum limits set by configuration. This achieves the same function as the “Set AC current” command. Gives you a freedom to send values between -100,0% and 100,0%. You do not need to know the motor limit parameters. This value must be between -100 and 100 and must be multiplied by 10 before sending."
};

const CanMessage Set_Relative_Brake_Current = {
    "Set relative brake current",
    0x06,
    "",
    "0-1",
    0,
    8,
    0.0f,
    0.0f,
    1.0f,
    "",
    "Targets the relative brake current of the motor. It will result negative torque relatively to the forward direction of the motor. This value must be between 0 and 100 and must be multiplied by 10 before sending Gives you a freedom to send values between 0% and 100,0%. You do not need to know the motor limit parameters. This value must be between 0 and 100 and has to be multiplied by 10 before sending"
};

const CanMessage Set_Digital_Output_1 = {
    "Set digital output",
    0x07,
    "Sets an output to HIGH or LOW",
    "0",
    0,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "Sets the digital output 1 to HIGH (1) or LOW (0) state"
};

const CanMessage Set_Digital_Output_2 = {
    "Set digital output",
    0x07,
    "Sets an output to HIGH or LOW",
    "0",
    1,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "Sets the digital output 2 to HIGH (1) or LOW (0) state"
};

const CanMessage Set_Digital_Output_3 = {
    "Set digital output",
    0x07,
    "Sets an output to HIGH or LOW",
    "0",
    2,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "Sets the digital output 3 to HIGH (1) or LOW (0) state"
};

const CanMessage Set_Digital_Output_4 = {
    "Set digital output",
    0x07,
    "Sets an output to HIGH or LOW",
    "0",
    3,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "Sets the digital output 4 to HIGH (1) or LOW (0) state"
};

const CanMessage Max_AC_Current = {
    "Max AC Current",
    0x08,
    "Limiting command",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "A_pk",
    "This value determines the maximum allowable drive current on the AC side. With this function you are able maximize the maximum torque on the motor. The value must be multiplied by 10 before sending."
};

const CanMessage Set_Maximum_AC_Brake_Current = {
    "Set maximum AC brake current",
    0x09,
    "Limiting command",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "A_pk",
    "This value sets the maximum allowable brake current on the AC side. This value must be multiplied by 10 before sending, only negative currents are accepted."
};

const CanMessage Max_DC_Current = {
    "Max DC Current",
    0x0A,
    "Limiting command",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "A",
    "This value determines the maximum allowable drive current on the DC side. With this command the BMS can limit the maximum allowable battery discharge current. The value has to be multiplied by 10 before sending."
};

const CanMessage Set_Maximum_DC_Brake_Current = {
    "Set maximum DC brake current",
    0x0B,
    "Limiting command",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "%",
    "This value determines the maximum allowable brake current on the DC side. With this command the BMS can limit the maximum allowable battery charge current. The value has to be multiplied by 10 before sending. Only negative currents are accepted."
};

const CanMessage Drive_Enable = {
    "Drive Enable",
    0x0C,
    "Limiting command",
    0,
    0,
    8,
    0,
    255,
    1,
    "#",
    "0: Drive not allowed 1: Drive allowed Only 0 and 1 values are accepted. Must be sent periodically to be enabled. Refer to chapter 4.3"
};

const CanMessage erpm = {
    "ERPM",
    0x10,
    "Motor speed",
    "0-3",
    0,
    8,
    -2147483648.0f,
    2147483647.0f,
    1.0f,
    "ERPM",
    "The current motor speed in ERPM. Equation: ERPM = Motor RPM * number of the motor pole pairs."
};

const CanMessage duty_cycle = {
    "Duty Cycle",
    0x11,
    "Current duty cycle",
    "0-1",
    0,
    8,
    -100.0f,
    100.0f,
    10.0f,
    "%",
    "The current duty cycle of the inverter. This is a signed parameter, and the sign represents the direction of the spinning. The value must be divided by 10 after receiving."
};

const CanMessage input_voltage = {
    "Input Voltage",
    0x12,
    "Current input voltage",
    "0-1",
    0,
    8,
    0.0f,
    6553.5f,
    10.0f,
    "V",
    "The current DC input voltage of the inverter. The value must be divided by 10 after receiving."
};

const CanMessage AC_current = {
    "AC Current",
    0x13,
    "Current motor AC current",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "A_pk",
    "The current motor AC current (peak, not RMS). This is a signed parameter, and the sign represents the direction of the torque which correlates with the motor AC current. The value must be divided by 10 after receiving."
};

const CanMessage DC_current = {
    "DC Current",
    0x14,
    "Current input DC current",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "A",
    "The current DC input current of the inverter. This is a signed parameter, and the sign represents the direction of the current (discharge/charge). The value must be divided by 10 after receiving."
};

const CanMessage RESERVED_1 = {
    "RESERVED_1",
    0x15,
    "",
    "",
    0,
    8,
    0.0f,
    0.0f,
    1.0f,
    "",
    "Reserved for future use"
};

const CanMessage controller_temperature = {
    "Controller Temperature",
    0x16,
    "Current controller temperature",
    "0-1",
    0,
    8,
    -40.0f,
    215.0f,
    1.0f,
    "°C",
    "The current temperature of the controller. The value must be divided by 1 after receiving."
};

const CanMessage motor_temperature = {
    "Motor Temperature",
    0x17,
    "Current motor temperature",
    "0-1",
    0,
    8,
    -40.0f,
    215.0f,
    1.0f,
    "°C",
    "The current temperature of the motor (if motor temperature sensor is used). The value must be divided by 1 after receiving."
};

const CanMessage fault_code = {
    "Fault Code",
    0x18,
    "Current fault code",
    "0-1",
    0,
    8,
    0.0f,
    65535.0f,
    1.0f,
    "#",
    "The current fault code of the controller. Refer to chapter 5 for the fault code list."
};

const CanMessage RESERVED_2 = {
    "RESERVED_2",
    0x19,
    "",
    "",
    0,
    8,
    0.0f,
    0.0f,
    1.0f,
    "",
    "Reserved for future use"
};

const CanMessage Id = {
    "Id",
    0x1A,
    "Current d-axis current",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "A_pk",
    "The current d-axis current of the motor. The value must be divided by 10 after receiving."
};

const CanMessage Iq = {
    "Iq",
    0x1B,
    "Current q-axis current",
    "0-1",
    0,
    8,
    -3276.8f,
    3276.7f,
    10.0f,
    "A_pk",
    "The current q-axis current of the motor. The value must be divided by 10 after receiving."
};

const CanMessage throttle_signal = {
    "Throttle Signal",
    0x1C,
    "Current throttle signal",
    "0-1",
    0,
    8,
    0.0f,
    100.0f,
    10.0f,
    "%",
    "The current throttle signal value. The value must be divided by 10 after receiving."
};

const CanMessage brake_signal = {
    "Brake Signal",
    0x1D,
    "Current brake signal",
    "0-1",
    0,
    8,
    0.0f,
    100.0f,
    10.0f,
    "%",
    "The current brake signal value. The value must be divided by 10 after receiving."
};

const CanMessage digital_input_1 = {
    "Digital Input 1",
    0x1E,
    "Current state of digital input 1",
    "0",
    0,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "The current state of the digital input 1. 0: LOW, 1: HIGH"
};

const CanMessage digital_input_2 = {
    "Digital Input 2",
    0x1E,
    "Current state of digital input 2",
    "0",
    1,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "The current state of the digital input 2. 0: LOW, 1: HIGH"
};

const CanMessage digital_input_3 = {
    "Digital Input 3",
    0x1E,
    "Current state of digital input 3",
    "0",
    2,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "The current state of the digital input 3. 0: LOW, 1: HIGH"
};

const CanMessage digital_input_4 = {
    "Digital Input 4",
    0x1E,
    "Current state of digital input 4",
    "0",
    3,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "The current state of the digital input 4. 0: LOW, 1: HIGH"
};

const CanMessage digital_input_1_2 = {
    "Digital Input 1 (alt)",
    0x1F,
    "Current state of digital input 1",
    "0",
    0,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "The current state of the digital input 1. 0: LOW, 1: HIGH"
};

const CanMessage digital_input_2_2 = {
    "Digital Input 2 (alt)",
    0x1F,
    "Current state of digital input 2",
    "0",
    1,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "The current state of the digital input 2. 0: LOW, 1: HIGH"
};

const CanMessage digital_input_3_2 = {
    "Digital Input 3 (alt)",
    0x1F,
    "Current state of digital input 3",
    "0",
    2,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "The current state of the digital input 3. 0: LOW, 1: HIGH"
};

const CanMessage digital_input_4_2 = {
    "Digital Input 4 (alt)",
    0x1F,
    "Current state of digital input 4",
    "0",
    3,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "The current state of the digital input 4. 0: LOW, 1: HIGH"
};

const CanMessage drive_enable = {
    "Drive Enable (alt)",
    0x20,
    "Current drive enable state",
    "0",
    0,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "The current drive enable state. 0: Drive not allowed, 1: Drive allowed"
};

const CanMessage capacitor_temp_limit = {
    "Capacitor Temp Limit",
    0x21,
    "Current capacitor temperature limit",
    "0-1",
    0,
    8,
    -40.0f,
    215.0f,
    1.0f,
    "°C",
    "The current capacitor temperature limit. The value must be divided by 1 after receiving."
};

const CanMessage DC_current_limit = {
    "DC Current Limit",
    0x22,
    "Current DC current limit",
    "0-1",
    0,
    8,
    0.0f,
    3276.7f,
    10.0f,
    "A",
    "The current DC current limit. The value must be divided by 10 after receiving."
};

const CanMessage drive_enable_limit = {
    "Drive Enable Limit",
    0x23,
    "Current drive enable limit",
    "0",
    0,
    8,
    0.0f,
    1.0f,
    1.0f,
    "#",
    "Drive enable limit active // 0: Drive enable limit inactive // Indicates whether the drive enable limitation is active or inactive. Used for software development purposes. For true indication of the drive state please use byte 3, bit 24 of this message."
};

const CanMessage igbt_acceleration_temperature_limit = {
    "IGBT Acceleration Temp Limit",
    0x24,
    "Current IGBT acceleration temperature limit",
    "0-1",
    0,
    8,
    -40.0f,
    215.0f,
    1.0f,
    "°C",
    "The current IGBT acceleration temperature limit. The value must be divided by 1 after receiving."
};

const CanMessage igbt_temperature_limit = {
    "IGBT Temperature Limit",
    0x25,
    "Current IGBT temperature limit",
    "0-1",
    0,
    8,
    -40.0f,
    215.0f,
    1.0f,
    "°C",
    "The current IGBT temperature limit. The value must be divided by 1 after receiving."
};

const CanMessage input_voltage_limit = {
    "Input Voltage Limit",
    0x26,
    "Current input voltage limit",
    "0-1",
    0,
    8,
    0.0f,
    6553.5f,
    10.0f,
    "V",
    "The current input voltage limit. The value must be divided by 10 after receiving."
};

const CanMessage motor_acceleration_temperature_limit = {
    "Motor Acceleration Temp Limit",
    0x27,
    "Current motor acceleration temperature limit",
    "0-1",
    0,
    8,
    -40.0f,
    215.0f,
    1.0f,
    "°C",
    "The current motor acceleration temperature limit. The value must be divided by 1 after receiving."
};

const CanMessage motor_temperature_limit = {
    "Motor Temperature Limit",
    0x28,
    "Current motor temperature limit",
    "0-1",
    0,
    8,
    -40.0f,
    215.0f,
    1.0f,
    "°C",
    "The current motor temperature limit. The value must be divided by 1 after receiving."
};

const CanMessage RPM_min_limit = {
    "RPM Min Limit",
    0x29,
    "Current minimum RPM limit",
    "0-1",
    0,
    8,
    -2147483648.0f,
    2147483647.0f,
    1.0f,
    "RPM",
    "The current minimum RPM limit. This is a signed parameter, and the sign represents the direction of the spinning. The value must be divided by 1 after receiving."
};

const CanMessage RPM_max_limit = {
    RPM max limit, ,
    41,8,0,1,1,"#",1: RPM max limit active // 0: RPM max limit inactive
    "RPM max limit active // 0: RPM max limit inactive"
};

const CanMessage power_limit = {
    "Power Limit",
    0x2B,
    "Current power limit",
    "0-1",
    0,
    8,
    0.0f,
    32767.0f,
    1.0f,
    "kW",
    "Power limit by configuration active // 0: Power limit by configuration inactive"
};

const CanMessage reserved_3 = {
    "Reserved 3",
    0x2C,
    "",
    "",
    0,
    8,
    0.0f,
    0.0f,
    1.0f,
    "",
    "set to 0"
};
const CanMessage reserved_4 = {
    "Reserved 4",
    0x2D,
    "",
    "",
    0,
    8,
    0.0f,
    0.0f,
    1.0f,
    "",
    "Reserved for future use"
};

const CanMessage CAN_map_version = {
    "CAN Map Version",
    0x2E,
    "Version of the CAN map",
    "0-1",
    0,
    8,
    0.0f,
    255.0f,
    1.0f,
    "#",
    "Indicates the CAN map version. For ex: 23 -> 2,3 (V2,3)"
};



const CanMessage[] ALL_MESSAGES = {
    Set_AC_Current,
    Set_Brake_Current,
    Set_ERPM,
    Set_Position,
    Set_Relative_Current,
    Set_Relative_Brake_Current,
    Set_Digital_Output_1,
    Set_Digital_Output_2,
    Set_Digital_Output_3,
    Set_Digital_Output_4,
    Max_AC_Current,
    Set_Maximum_AC_Brake_Current,
    Max_DC_Current,
    Set_Maximum_DC_Brake_Current,
    Drive_Enable


};

const int NUM_MESSAGES = sizeof(ALL_MESSAGES);


float BDRCANLib::we_love_jaden_lee() {
    Serial.println("We love Jaden Lee!");
    return 42.0f; // The answer to life, the universe, and everything
}