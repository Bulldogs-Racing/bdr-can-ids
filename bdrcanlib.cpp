#include "Arduino.h"
#include "bdrcanlib.h"

BDRCANLib::BDRCANLib()
{
    // constructor body left intentionally empty — real init can go in begin()
}
messageStruct BDRCANLib::createMessage(uint32_t id, const uint8_t *data, uint8_t length)
{
    messageStruct m;
    m.id = id;
    m.length = length;
    if (data && length > 0)
    {
        memcpy(m.data, data, length);
    }
    else
    {
        memset(m.data, 0, sizeof(m.data));
    }
    return m;
}

float BDRCANLib::conv_to_dec(const String &s)
{
    String tmp = s;
    tmp.replace(',', '.');
    return tmp.toFloat();
}

static constexpr int MESSAGE_SIZE = 8; // default size of the CanMessage structure

/*
 * Define every CAN ID used in the system.
 * Add or modify as needed for your application.
 */
static constexpr CanMessage ac_current = {
    "Set AC Current",
    0x01,
    "ac_current",
    0 - 1,
    1,
    8,
    -3276, 8,
    3276, 7,
    10,
    "Aₚₖ",
    "This command sets the target motor AC current (peak, not RMS). When the controller receives this message, it automatically switches to current control mode. This value must not be above the limits of the inverter and must be multiplied by 10 before sending. This is a signed parameter, and the sign represents the direction of the torque which correlates with the motor AC current. (For the correlation, please refer to the motor parameters)"};
static constexpr CanMessage NOT_USED_0 = {
    "",
    0x01,
    "NOT_USED",
    2 - 7,
    16,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 2-byte DLC."};
static constexpr CanMessage target_brake_current = {
    "Set Brake current",
    0x02,
    "target_brake_current",
    0 - 1,
    0,
    8,
    -3276, 8,
    3276, 7,
    10,
    "Aₚₖ",
    "Targets the brake current of the motor. It will result negative torque relatively to the forward direction of the motor. This value must be multiplied by 10 before sending, only positive currents are accepted."};
static constexpr CanMessage NOT_USED_1 = {
    "",
    0x02,
    "NOT_USED",
    2 - 7,
    16,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 2-byte DLC."};
static constexpr CanMessage Set_speed_(ERPM) = {
    "Set ERPM",
    0x03,
    "Set_speed_(ERPM)",
    0 - 3,
    0,
    8,
    -2147483648,
    2147483647,
    1,
    "ERPM",
    "This command enables the speed control of the motor with a target ERPM. This is a signed parameter, and the sign represents the direction of the spinning. For better operation you need to tune the PID of speed control. Equation: ERPM = Motor RPM * number of the motor pole pairs."};
static constexpr CanMessage NOT_USED_2 = {
    "",
    0x03,
    "NOT_USED",
    45754,
    32,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 4-byte DLC."};
static constexpr CanMessage Target_position = {
    "Set Position",
    0x04,
    "Target_position",
    0 - 1,
    0,
    8,
    -3276, 8,
    3276, 7,
    10,
    "degree",
    "This value targets the desired position of the motor in degrees. This command is used to hold a position of the motor. This feature is enabled only if encoder is used as position sensor. The value has to be multiplied by 10 before sending."};
static constexpr CanMessage NOT_USED_3 = {
    "",
    0x04,
    "NOT_USED",
    45695,
    16,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 2-byte DLC."};
static constexpr CanMessage Set_relative_current = {
    "Set Relative current",
    0x05,
    "Set_relative_current",
    0 - 1,
    0,
    8,
    -3276, 8,
    3276, 7,
    10,
    "%",
    "This command sets a relative AC current to the minimum and maximum limits set by configuration. This achieves the same function as the “Set AC current” command. Gives you a freedom to send values between -100,0% and 100,0%. You do not need to know the motor limit parameters. This value must be between -100 and 100 and must be multiplied by 10 before sending."};
static constexpr CanMessage NOT_USED_4 = {
    "",
    0x05,
    "NOT_USED",
    45695,
    32,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 2-byte DLC."};
static constexpr CanMessage Set_Relative_Brake_Current = {
    "Set relative brake current",
    0x06,
    "",
    0 - 1,
    0,
    8,
    ,
    ,
    ,
    "",
    "Targets the relative brake current of the motor. It will result negative torque relatively to the forward direction of the motor. This value must be between 0 and 100 and must be multiplied by 10 before sending Gives you a freedom to send values between 0% and 100,0%. You do not need to know the motor limit parameters. This value must be between 0 and 100 and has to be multiplied by 10 before sending"};
static constexpr CanMessage NOT_USED_5 = {
    "",
    0x06,
    "NOT_USED",
    45695,
    16,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 2-byte DLC."};
static constexpr CanMessage Set_Digital_Output_1 = {
    "Set digital output ",
    0x07,
    "Sets_an_output_to_HIGH_or_LOW",
    0,
    0,
    8,
    0,
    1,
    1,
    "#",
    "Sets the digital output 1 to HIGH (1) or LOW (0) state"};
static constexpr CanMessage Set_Digital_Output_2 = {
    "Set digital output ",
    0x07,
    "Sets_an_output_to_HIGH_or_LOW",
    0,
    1,
    8,
    0,
    1,
    1,
    "#",
    "Sets the digital output 2 to HIGH (1) or LOW (0) state"};
static constexpr CanMessage Set_Digital_Output_3 = {
    "Set digital output ",
    0x07,
    "Sets_an_output_to_HIGH_or_LOW",
    0,
    2,
    8,
    0,
    1,
    1,
    "#",
    "Sets the digital output 3 to HIGH (1) or LOW (0) state"};
static constexpr CanMessage Set_Digital_Output_4 = {
    "Set digital output ",
    0x07,
    "Sets_an_output_to_HIGH_or_LOW",
    0,
    3,
    8,
    0,
    1,
    1,
    "#",
    "Sets the digital output 4 to HIGH (1) or LOW (0) state"};
static constexpr CanMessage Limiting_command = {
    "Max AC Current",
    0x08,
    "Limiting_command",
    0 - 1,
    0,
    8,
    -3276, 8,
    3276, 7,
    10,
    "Aₚₖ",
    "This value determines the maximum allowable drive current on the AC side. With this function you are able maximize the maximum torque on the motor. The value must be multiplied by 10 before sending."};
static constexpr CanMessage NOT_USED_6 = {
    "",
    0x08,
    "NOT_USED",
    45695,
    32,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 2-byte DLC."};
static constexpr CanMessage Limiting_command = {
    "Set maximum AC brake current",
    0x09,
    "Limiting_command",
    0 - 1,
    0,
    8,
    -3276, 8,
    3276, 7,
    10,
    "Aₚₖ",
    "This value sets the maximum allowable brake current on the AC side. This value must be multiplied by 10 before sending, only negative currents are accepted."};
static constexpr CanMessage NOT_USED_7 = {
    "",
    0x09,
    "NOT_USED",
    45695,
    16,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 2-byte DLC."};
static constexpr CanMessage Limiting_command = {
    "Max DC Current",
    0x0A,
    "Limiting_command",
    0 - 1,
    0,
    8,
    -3276, 8,
    3276, 7,
    10,
    "A",
    "This value determines the maximum allowable drive current on the DC side. With this command the BMS can limit the maximum allowable battery discharge current. The value has to be multiplied by 10 before sending."};
static constexpr CanMessage NOT_USED_8 = {
    "",
    0x0A,
    "NOT_USED",
    45695,
    32,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 2-byte DLC."};
static constexpr CanMessage Limiting_command = {
    "Set maximum DC brake current",
    0x0B,
    "Limiting_command",
    0 - 1,
    0,
    8,
    -3276, 8,
    3276, 7,
    10,
    "%",
    "This value determines the maximum allowable brake current on the DC side. With this command the BMS can limit the maximum allowable battery charge current. The value has to be multiplied by 10 before sending. Only negative currents are accepted."};
static constexpr CanMessage NOT_USED_9 = {
    "",
    0x0B,
    "NOT_USED",
    45695,
    16,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 2-byte DLC."};
static constexpr CanMessage Limiting_command = {
    "Drive Enable",
    0x0C,
    "Limiting_command",
    0,
    0,
    8,
    0,
    255,
    1,
    "#",
    "0: Drive not allowed 1: Drive allowed Only 0 and 1 values are accepted. Must be sent periodically to be enabled. Refer to chapter 4.3"};
static constexpr CanMessage NOT_USED_10 = {
    "",
    0x0C,
    "NOT_USED",
    45664,
    8,
    8,
    -,
    -,
    -,
    "-",
    "Not relevant to the command. Fill with FFs or use 1-byte DLC."};

/////////////////////////////////////////////////////////////////////////////////////////

// Additional CAN messages
static constexpr CanMessage erpm = {
    "general data 1",
    0x20,
    "erpm",
    0 - 3,
    0,
    8,
    -2147483648,
    2147483648,
    1,
    "ERPM",
    "Electrical RPM Equation: ERPM = Motor RPM * number of the motor pole pairs."};
static constexpr CanMessage duty_cycle = {
    "",
    0x20,
    "duty_cycle",
    4 - 5,
    32,
    8,
    -3276, 8,
    3276, 7,
    10,
    "%",
    "The controller duty cycle. The sign of this value will represent whether the motor is running(positive) current or regenerating (negative) current."};
static constexpr CanMessage input_voltage = {
    "",
    0x20,
    "input_voltage",
    6 - 7,
    48,
    8,
    -32768,
    32767,
    1,
    "V",
    "Input voltage is the DC voltage."};
static constexpr CanMessage AC_current = {
    "general data 2",
    0x21,
    "AC_current",
    0 - 1,
    0,
    8,
    -3276, 8,
    3276, 7,
    10,
    "Aₚₖ",
    "The motor current. The sign of this value represents whether the motor is running(positive) current or regenerating (negative) current."};
static constexpr CanMessage DC_current = {
    "",
    0x21,
    "DC_current",
    45691,
    16,
    8,
    -3276, 8,
    3276, 7,
    10,
    "Aₚₖ",
    "DC Current: Current on DC side. The sign of this value represents whether the motor is running(positive) current or regenerating (negative) current."};
static constexpr CanMessage RESERVED_0 = {
    "",
    0x21,
    "RESERVED",
    45754,
    32,
    8,
    -,
    -,
    -,
    "-",
    "Filled with FF’s. For future use."};
static constexpr CanMessage controller_temperature = {
    "general data 3",
    0x22,
    "controller_temperature",
    0 - 1,
    0,
    8,
    -3276, 8,
    3276, 7,
    10,
    "°C",
    "Temperature of the inverter semiconductors."};
static constexpr CanMessage motor_temperature = {
    "",
    0x22,
    "motor_temperature",
    2 - 3,
    16,
    8,
    -3276, 8,
    3276, 7,
    10,
    "°C",
    "Temperature of the motor measured by the inverter"};
static constexpr CanMessage fault_code = {
    "",
    0x22,
    "fault_code",
    1 - 3,
    32,
    8,
    0,
    255,
    1,
    "#",
    "fault code (see fault code chart)"};
static constexpr CanMessage RESERVED_1 = {
    "",
    0x22,
    "RESERVED",
    45784,
    40,
    8,
    -,
    -,
    -,
    "-",
    "Filled with FF’s. For future use."};
static constexpr CanMessage Id = {
    "general data 4",
    0x23,
    "Id",
    0 - 3,
    0,
    8,
    -2147483648,
    2147483648,
    100,
    "Aₚₖ",
    "FOC algorithm component Id."};
static constexpr CanMessage Iq = {
    "",
    0x23,
    "Iq",
    45754,
    32,
    8,
    -2147483648,
    2147483648,
    100,
    "Aₚₖ",
    "FOC algorithm component Iq"};
static constexpr CanMessage throttle_signal = {
    "general data 5",
    0x24,
    "throttle_signal",
    0,
    0,
    8,
    -128,
    127,
    1,
    "%",
    "Throttle signal derived from analog inputs or CAN2"};
static constexpr CanMessage brake_signal = {
    "",
    0x24,
    "brake_signal",
    1,
    8,
    8,
    -128,
    127,
    1,
    "%",
    "Brake signal derived from analog inputs or CAN2"};
static constexpr CanMessage digital_input_1 = {
    "",
    0x24,
    "digital_input_1",
    2,
    16,
    8,
    0,
    1,
    1,
    "#",
    "1: Digital input is active // 0: Digital input is inactive"};
static constexpr CanMessage digital_input_2 = {
    "",
    0x24,
    "digital_input_2",
    ,
    17,
    8,
    0,
    1,
    1,
    "#",
    ""};
static constexpr CanMessage digital_input_3 = {
    "",
    0x24,
    "digital_input_3",
    ,
    18,
    8,
    0,
    1,
    1,
    "#",
    ""};
static constexpr CanMessage digital_input_4 = {
    "",
    0x24,
    "digital_input_4",
    ,
    19,
    8,
    0,
    1,
    1,
    "#",
    ""};
static constexpr CanMessage digital_input_1 = {
    "",
    0x24,
    "digital_input_1",
    ,
    20,
    8,
    0,
    1,
    1,
    "#",
    "1: Digital output is active // 0: Digital output is inactive"};
static constexpr CanMessage digital_input_2 = {
    "",
    0x24,
    "digital_input_2",
    ,
    21,
    8,
    0,
    1,
    1,
    "#",
    ""};
static constexpr CanMessage digital_input_3 = {
    "",
    0x24,
    "digital_input_3",
    ,
    22,
    8,
    0,
    1,
    1,
    "#",
    ""};
static constexpr CanMessage digital_input_4 = {
    "",
    0x24,
    "digital_input_4",
    ,
    23,
    8,
    0,
    1,
    1,
    "#",
    ""};
static constexpr CanMessage drive_enable = {
    "",
    0x24,
    "drive_enable",
    3,
    24,
    8,
    0,
    1,
    1,
    "#",
    "1: Drive enabled // 0: Drive disabled // Drive can be enabled/disbled by the digital input or/and via //CAN2 interface"};
static constexpr CanMessage capacitor_temp_limit = {
    "",
    0x24,
    "capacitor_temp_limit",
    4,
    32,
    8,
    0,
    1,
    1,
    "#",
    "1: Capacitor temperature limit active // 0: Capacitor temperature limit inactive. The inverter can limit the output power to not to overheat the internal capacitors. (only valid HW version 3.6 or newer)"};
static constexpr CanMessage DC_current_limit = {
    "",
    0x24,
    "DC_current_limit",
    ,
    33,
    8,
    0,
    1,
    1,
    "#",
    "1: DC current limit active // 0: DC current limit inactive"};
static constexpr CanMessage drive_enable_limit = {
    "",
    0x24,
    "drive_enable_limit",
    ,
    34,
    8,
    0,
    1,
    1,
    "#",
    "1: Drive enable limit active // 0: Drive enable limit inactive // Indicates whether the drive enable limitation is active or inactive. Used for software development purposes. For true indication of the drive state please use byte 3, bit 24 of this message."};
static constexpr CanMessage igbt_acceleration_temperature_limit = {
    "",
    0x24,
    "igbt_acceleration_temperature_limit",
    ,
    35,
    8,
    0,
    1,
    1,
    "#",
    "1: IGBT acceleration limit active // 0: IGBT acceleration limit inactive"};
static constexpr CanMessage igbt_temperature_limit = {
    "",
    0x24,
    "igbt_temperature_limit",
    ,
    36,
    8,
    0,
    1,
    1,
    "#",
    "1: IGBT temperature limit active // 0: IGBT temperature limit inactive"};
static constexpr CanMessage input_voltage_limit = {
    "",
    0x24,
    "input_voltage_limit",
    ,
    37,
    8,
    0,
    1,
    1,
    "#",
    "1: Input voltage limit active // 0: Input voltage limit inactive"};
static constexpr CanMessage motor_acceleration_temperature_limit = {
    "",
    0x24,
    "motor_acceleration_temperature_limit",
    ,
    38,
    8,
    0,
    1,
    1,
    "#",
    "1: Motor acceleration temperature limit active // 0: Motor acceleration temperature limit inactive"};
static constexpr CanMessage motor_temperature_limit = {
    "",
    0x24,
    "motor_temperature_limit",
    ,
    39,
    8,
    0,
    1,
    1,
    "#",
    "1: Motor temperature limit active // 0: Motor temperature limit inactive"};
static constexpr CanMessage RPM_min_limit = {
    "",
    0x24,
    "RPM_min_limit",
    5,
    40,
    8,
    0,
    1,
    1,
    "#",
    "1: RPM min limit active // 0: RPM min limit inactive"};
static constexpr CanMessage RPM_max_limit = {
    "",
    0x24,
    "RPM_max_limit",
    ,
    41,
    8,
    0,
    1,
    1,
    "#",
    "1: RPM max limit active // 0: RPM max limit inactive"};
static constexpr CanMessage power_limit = {
    "",
    0x24,
    "power_limit",
    ,
    42,
    8,
    0,
    1,
    1,
    "#",
    "1: Power limit by configuration active // 0: Power limit by configuration inactive"};
static constexpr CanMessage reserved_3 = {
    "",
    0x24,
    "reserved",
    ,
    43 - 47,
    8,
    0,
    1,
    1,
    "#",
    "Set to 0."};
static constexpr CanMessage reserved_4 = {
    "",
    0x24,
    "reserved",
    6,
    48,
    8,
    -,
    -,
    -,
    "-",
    "Filled with FF’s. For future use."};
static constexpr CanMessage CAN_map_version = {
    "",
    0x24,
    "CAN_map_version",
    7,
    56,
    8,
    0,
    255,
    1,
    "#",
    "Indicates the CAN map version. For ex: 23 -> 2,3 (V2,3)"};

static constexpr CanMessage[] ALL_MESSAGES = {
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

static constexpr int NUM_MESSAGES = sizeof(ALL_MESSAGES);

float BDRCANLib::we_love_jaden_lee()
{
    Serial.println("We love Jaden Lee!");
    return 42.0f; // The answer to life, the universe, and everything
}