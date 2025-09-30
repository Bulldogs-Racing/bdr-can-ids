#include "Arduino.h"
#include "bdrcanlib.h"

BDRCANLib::BDRCANLib() {
    // constructor body left intentionally empty — real init can go in begin()
}

bdrcanlib::messageStruct BDRCANLib::createMessage(uint32_t id, const uint8_t* data, uint8_t length) {
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
CanMessage Set_AC_Current = {
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

CanMessage Set_Brake_Current = {
    "Set Brake current",
    0x02,
    "target brake current",
    0-1,
    0,
    8,
    "-3276,8",
    "3276,7",
    10,
    "Aₚₖ",
    "Targets the brake current of the motor. It will result negative torque relatively to the forward direction of the motor. This value must be multiplied by 10 before sending, only positive currents are accepted."
};

CanMessage Set_ERPM = {
    "Set ERPM",
    0x03,
    "Set speed (ERPM)",
    0-3,
    0,
    8,
    -2147483648,
    2147483647,
    1,
    "ERPM",
    "This command enables the speed control of the motor with a target ERPM. This is a signed parameter, and the sign represents the direction of the spinning. For better operation you need to tune the PID of speed control. Equation: ERPM = Motor RPM * number of the motor pole pairs."
};

CanMessage Set_Position = {
    "Set Position",
    0x04,
    "Target position",
    0-1,
    0,
    8,
    "-3276,8",
    "3276,7",
    10,
    "degree",
    "This value targets the desired position of the motor in degrees. This command is used to hold a position of the motor. This feature is enabled only if encoder is used as position sensor. The value has to be multiplied by 10 before sending."
};

CanMessage Set_Relative_Current = {
    "Set Relative current",
    0x05,
    "Set relative current",
    0-1,
    0,
    8,
    "-3276,8",
    "3276,7",
    10,
    "%",
    "This command sets a relative AC current to the minimum and maximum limits set by configuration. This achieves the same function as the “Set AC current” command. Gives you a freedom to send values between -100,0% and 100,0%. You do not need to know the motor limit parameters. This value must be between -100 and 100 and must be multiplied by 10 before sending."
};

CanMessage Set_Relative_Brake_Current = {
    "Set relative brake current",
    0x06,
    "",
    0-1,
    0,
    8,
    "",
    "",
    "",
    "",
    "Targets the relative brake current of the motor. It will result negative torque relatively to the forward direction of the motor. This value must be between 0 and 100 and must be multiplied by 10 before sending Gives you a freedom to send values between 0% and 100,0%. You do not need to know the motor limit parameters. This value must be between 0 and 100 and has to be multiplied by 10 before sending"
};

CanMessage Set_Digital_Output_1 = {
    "Set digital output",
    0x07,
    "Sets an output to HIGH or LOW",
    0,
    0,
    8,
    0,
    1,
    1,
    "#",
    "Sets the digital output 1 to HIGH (1) or LOW (0) state"
};

CanMessage Set_Digital_Output_2 = {
    "Set digital output",
    0x07,
    "Sets an output to HIGH or LOW",
    0,
    1,
    8,
    0,
    1,
    1,
    "#",
    "Sets the digital output 2 to HIGH (1) or LOW (0) state"
};

CanMessage Set_Digital_Output_3 = {
    "Set digital output",
    0x07,
    "Sets an output to HIGH or LOW",
    0,
    2,
    8,
    0,
    1,
    1,
    "#",
    "Sets the digital output 3 to HIGH (1) or LOW (0) state"
};

CanMessage Set_Digital_Output_4 = {
    "Set digital output",
    0x07,
    "Sets an output to HIGH or LOW",
    0,
    3,
    8,
    0,
    1,
    1,
    "#",
    "Sets the digital output 4 to HIGH (1) or LOW (0) state"
};

CanMessage Max_AC_Current = {
    "Max AC Current",
    0x08,
    "Limiting command",
    0-1,
    0,
    8,
    "-3276,8",
    "3276,7",
    10,
    "Aₚₖ",
    "This value determines the maximum allowable drive current on the AC side. With this function you are able maximize the maximum torque on the motor. The value must be multiplied by 10 before sending."
};

CanMessage Set_Maximum_AC_Brake_Current = {
    "Set maximum AC brake current",
    0x09,
    "Limiting command",
    0-1,
    0,
    8,
    "-3276,8",
    "3276,7",
    10,
    "Aₚₖ",
    "This value sets the maximum allowable brake current on the AC side. This value must be multiplied by 10 before sending, only negative currents are accepted."
};

CanMessage Max_DC_Current = {
    "Max DC Current",
    0x0A,
    "Limiting command",
    0-1,
    0,
    8,
    "-3276,8",
    "3276,7",
    10,
    "A",
    "This value determines the maximum allowable drive current on the DC side. With this command the BMS can limit the maximum allowable battery discharge current. The value has to be multiplied by 10 before sending."
};

CanMessage Set_Maximum_DC_Brake_Current = {
    "Set maximum DC brake current",
    0x0B,
    "Limiting command",
    0-1,
    0,
    8,
    "-3276,8",
    "3276,7",
    10,
    "%",
    "This value determines the maximum allowable brake current on the DC side. With this command the BMS can limit the maximum allowable battery charge current. The value has to be multiplied by 10 before sending. Only negative currents are accepted."
};

CanMessage Drive_Enable = {
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


