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

uint32_t* BDRCANLib::getAllCANIDs() {
    static uint32_t ids[] = {
        Set_AC_Current.id,
        Set_Brake_Current.id,
        Set_ERPM.id,
        Set_Position.id,
        Set_Relative_Current.id,
        Set_Relative_Brake_Current.id,
        Set_Digital_Output_1.id,
        Set_Digital_Output_2.id,
        Set_Digital_Output_3.id,
        Set_Digital_Output_4.id,
        Max_AC_Current.id,
        Set_Maximum_AC_Brake_Current.id,
        Max_DC_Current.id,
        Set_Maximum_DC_Brake_Current.id,
        Drive_Enable.id,
        // Orion BMS CAN IDs
        relays_status.id,
        max_cells_supported_count.id,
        populated_cell_count.id,
        pack_charge_current_limit.id,
        pack_discharge_current_limit.id,
        signed_pack_current.id,
        unsigned_pack_current.id,
        pack_voltage.id,
        pack_open_voltage.id,
        pack_state_of_charge.id,
        pack_amphours.id,
        pack_resistance.id,
        pack_depth_of_discharge.id,
        pack_health.id,
        pack_summed_voltage.id,
        total_pack_cycles.id,
        highest_pack_temperature.id,
        lowest_pack_temperature.id,
        avg_pack_temperature.id,
        heatsink_temperature_sensor.id,
        fan_speed.id,
        requested_fan_speed.id,
        low_cell_voltage.id,
        low_cell_voltage_id.id,
        high_cell_voltage.id,
        high_cell_voltage_id.id,
        avg_cell_voltage.id,
        low_opencell_voltage.id,
        low_opencell_voltage_id.id,
        high_opencell_voltage.id,
        high_opencell_voltage_id.id,
        avg_opencell_voltage.id,
        low_cell_resistance.id,
        low_cell_resistance_id.id,
        high_cell_resistance.id,
        high_cell_resistance_id.id,
        avg_cell_resistance.id,
        input_power_supply_voltage.id,
        fan_voltage.id,
        // Cell voltage arrays
        cell_voltages_1_12.id,
        cell_voltages_13_24.id,
        cell_voltages_25_36.id,
        cell_voltages_37_48.id,
        cell_voltages_49_60.id,
        cell_voltages_61_72.id,
        cell_voltages_73_84.id,
        cell_voltages_85_96.id,
        cell_voltages_97_108.id,
        cell_voltages_109_120.id,
        cell_voltages_121_132.id,
        cell_voltages_133_144.id,
        cell_voltages_145_156.id,
        cell_voltages_157_168.id,
        cell_voltages_169_180.id,
        // Opencell voltage arrays
        opencell_voltages_1_12.id,
        opencell_voltages_13_24.id,
        opencell_voltages_25_36.id,
        opencell_voltages_37_48.id,
        opencell_voltages_49_60.id,
        opencell_voltages_61_72.id,
        opencell_voltages_73_84.id,
        opencell_voltages_85_96.id,
        opencell_voltages_97_108.id,
        opencell_voltages_109_120.id,
        opencell_voltages_121_132.id,
        opencell_voltages_133_144.id,
        opencell_voltages_145_156.id,
        opencell_voltages_157_168.id,
        opencell_voltages_169_180.id,
        // Internal resistance arrays
        internal_resistances_1_12.id,
        internal_resistances_13_24.id,
        internal_resistances_25_36.id,
        internal_resistances_37_48.id,
        internal_resistances_49_60.id,
        internal_resistances_61_72.id,
        internal_resistances_73_84.id,
        internal_resistances_85_96.id,
        internal_resistances_97_108.id,
        internal_resistances_109_120.id,
        internal_resistances_121_132.id,
        internal_resistances_133_144.id,
        internal_resistances_145_156.id,
        internal_resistances_157_168.id,
        internal_resistances_169_180.id
    };
    return ids;
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

// Orion BMS CAN messages
const CanMessage relays_status = {
    "Relays Status",
    0xF004,
    "General Broadcast To Network",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    1.0f,
    "",
    "General Broadcast To Network: 0x7DF 8 01 3E 00 00 00 00 00 00"
};

const CanMessage max_cells_supported_count = {
    "Max Cells Supported Count",
    0xF006,
    "",
    "0",
    0,
    1,
    0.0f,
    255.0f,
    1.0f,
    "",
    ""
};

const CanMessage populated_cell_count = {
    "Populated Cell Count",
    0xF007,
    "",
    "0",
    0,
    1,
    0.0f,
    255.0f,
    1.0f,
    "",
    ""
};

const CanMessage pack_charge_current_limit = {
    "Pack Charge Current Limit",
    0xF00A,
    "Amps",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    1.0f,
    "Amps",
    ""
};

const CanMessage pack_discharge_current_limit = {
    "Pack Discharge Current Limit",
    0xF00B,
    "Amps",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    1.0f,
    "Amps",
    ""
};

const CanMessage signed_pack_current = {
    "Signed Pack Current",
    0xF00C,
    "Amps",
    "0",
    0,
    2,
    -32767.0f,
    32767.0f,
    0.1f,
    "Amps",
    ""
};

const CanMessage unsigned_pack_current = {
    "Unsigned Pack Current",
    0xF015,
    "Amps",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    0.1f,
    "Amps",
    "NOTE: To get actual amperage, subtract 32767 from the value."
};

const CanMessage pack_voltage = {
    "Pack Voltage",
    0xF00D,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    0.1f,
    "Volts",
    ""
};

const CanMessage pack_open_voltage = {
    "Pack Open Voltage",
    0xF00E,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    0.1f,
    "Volts",
    ""
};

const CanMessage pack_state_of_charge = {
    "Pack State of Charge",
    0xF00F,
    "%",
    "0",
    0,
    1,
    0.0f,
    100.0f,
    0.5f,
    "%",
    ""
};

const CanMessage pack_amphours = {
    "Pack Amphours",
    0xF010,
    "Amphours",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    0.1f,
    "Amphours",
    ""
};

const CanMessage pack_resistance = {
    "Pack Resistance",
    0xF011,
    "mOhm",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    0.01f,
    "mOhm",
    ""
};

const CanMessage pack_depth_of_discharge = {
    "Pack Depth of Discharge",
    0xF012,
    "%",
    "0",
    0,
    1,
    0.0f,
    100.0f,
    0.5f,
    "%",
    ""
};

const CanMessage pack_health = {
    "Pack Health",
    0xF013,
    "%",
    "0",
    0,
    1,
    0.0f,
    100.0f,
    1.0f,
    "%",
    ""
};

const CanMessage pack_summed_voltage = {
    "Pack Summed Voltage",
    0xF014,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    0.01f,
    "Volts",
    ""
};

const CanMessage total_pack_cycles = {
    "Total Pack Cycles",
    0xF018,
    "#",
    "0",
    0,
    2,
    0.0f,
    65535.0f,
    1.0f,
    "#",
    ""
};

const CanMessage highest_pack_temperature = {
    "Highest Pack Temperature",
    0xF028,
    "Celsius",
    "0",
    0,
    1,
    -40.0f,
    80.0f,
    1.0f,
    "Celsius",
    ""
};

const CanMessage lowest_pack_temperature = {
    "Lowest Pack Temperature",
    0xF029,
    "Celsius",
    "0",
    0,
    1,
    -40.0f,
    80.0f,
    1.0f,
    "Celsius",
    ""
};

const CanMessage avg_pack_temperature = {
    "Avg. Pack Temperature",
    0xF02A,
    "Celsius",
    "0",
    0,
    1,
    -40.0f,
    80.0f,
    1.0f,
    "Celsius",
    ""
};

const CanMessage heatsink_temperature_sensor = {
    "Heatsink Temperature Sensor",
    0xF02D,
    "Celsius",
    "0",
    0,
    1,
    -40.0f,
    80.0f,
    1.0f,
    "Celsius",
    ""
};

const CanMessage fan_speed = {
    "Fan Speed",
    0xF02B,
    "#",
    "0",
    0,
    1,
    0.0f,
    6.0f,
    1.0f,
    "#",
    ""
};

const CanMessage requested_fan_speed = {
    "Requested Fan Speed",
    0xF02C,
    "#",
    "0",
    0,
    1,
    0.0f,
    6.0f,
    1.0f,
    "#",
    ""
};

const CanMessage low_cell_voltage = {
    "Low Cell Voltage",
    0xF032,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    ""
};

const CanMessage low_cell_voltage_id = {
    "Low Cell Voltage ID (Cell Num)",
    0xF03E,
    "#",
    "0",
    0,
    2,
    0.0f,
    180.0f,
    1.0f,
    "#",
    ""
};

const CanMessage high_cell_voltage = {
    "High Cell Voltage",
    0xF033,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    ""
};

const CanMessage high_cell_voltage_id = {
    "High Cell Voltage ID (Cell Num)",
    0xF03D,
    "#",
    "0",
    0,
    2,
    0.0f,
    180.0f,
    1.0f,
    "#",
    ""
};

const CanMessage avg_cell_voltage = {
    "Avg. Cell Voltage",
    0xF034,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    ""
};

const CanMessage low_opencell_voltage = {
    "Low Opencell Voltage",
    0xF035,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    ""
};

const CanMessage low_opencell_voltage_id = {
    "Low Opencell Voltage ID (Cell Num)",
    0xF040,
    "#",
    "0",
    0,
    2,
    0.0f,
    180.0f,
    1.0f,
    "#",
    ""
};

const CanMessage high_opencell_voltage = {
    "High Opencell Voltage",
    0xF036,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    ""
};

const CanMessage high_opencell_voltage_id = {
    "High Opencell Voltage ID (Cell Num)",
    0xF03F,
    "#",
    "0",
    0,
    2,
    0.0f,
    180.0f,
    1.0f,
    "#",
    ""
};

const CanMessage avg_opencell_voltage = {
    "Avg. Opencell Voltage",
    0xF037,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    ""
};

const CanMessage low_cell_resistance = {
    "Low Cell Resistance",
    0xF038,
    "mOhm",
    "0",
    0,
    2,
    0.0f,
    655.35f,
    0.01f,
    "mOhm",
    ""
};

const CanMessage low_cell_resistance_id = {
    "Low Cell Resistance ID (Cell Num)",
    0xF042,
    "#",
    "0",
    0,
    2,
    0.0f,
    180.0f,
    1.0f,
    "#",
    ""
};

const CanMessage high_cell_resistance = {
    "High Cell Resistance",
    0xF039,
    "mOhm",
    "0",
    0,
    2,
    0.0f,
    655.35f,
    0.01f,
    "mOhm",
    ""
};

const CanMessage high_cell_resistance_id = {
    "High Cell Resistance ID (Cell Num)",
    0xF041,
    "#",
    "0",
    0,
    2,
    0.0f,
    180.0f,
    1.0f,
    "#",
    ""
};

const CanMessage avg_cell_resistance = {
    "Avg. Cell Resistance",
    0xF03A,
    "mOhm",
    "0",
    0,
    2,
    0.0f,
    655.35f,
    0.01f,
    "mOhm",
    ""
};

const CanMessage input_power_supply_voltage = {
    "Input Power Supply Voltage",
    0xF046,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    35.0f,
    0.1f,
    "Volts",
    ""
};

const CanMessage fan_voltage = {
    "Fan Voltage",
    0xF049,
    "Volts",
    "0",
    0,
    2,
    0.0f,
    15.0f,
    0.01f,
    "Volts",
    ""
};

// Cell voltage arrays (15 messages for cells 1-180)
const CanMessage cell_voltages_1_12 = {
    "Cell Voltages (Cells 1-12)",
    0xF100,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_13_24 = {
    "Cell Voltages (Cells 13-24)",
    0xF101,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_25_36 = {
    "Cell Voltages (Cells 25-36)",
    0xF102,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_37_48 = {
    "Cell Voltages (Cells 37-48)",
    0xF103,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_49_60 = {
    "Cell Voltages (Cells 49-60)",
    0xF104,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_61_72 = {
    "Cell Voltages (Cells 61-72)",
    0xF105,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_73_84 = {
    "Cell Voltages (Cells 73-84)",
    0xF106,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_85_96 = {
    "Cell Voltages (Cells 85-96)",
    0xF107,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_97_108 = {
    "Cell Voltages (Cells 97-108)",
    0xF108,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_109_120 = {
    "Cell Voltages (Cells 109-120)",
    0xF109,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_121_132 = {
    "Cell Voltages (Cells 121-132)",
    0xF10A,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_133_144 = {
    "Cell Voltages (Cells 133-144)",
    0xF10B,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_145_156 = {
    "Cell Voltages (Cells 145-156)",
    0xF10C,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_157_168 = {
    "Cell Voltages (Cells 157-168)",
    0xF10D,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage cell_voltages_169_180 = {
    "Cell Voltages (Cells 169-180)",
    0xF10E,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

// Opencell voltage arrays (15 messages for cells 1-180)
const CanMessage opencell_voltages_1_12 = {
    "Opencell Voltages (Cells 1-12)",
    0xF300,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_13_24 = {
    "Opencell Voltages (Cells 13-24)",
    0xF301,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_25_36 = {
    "Opencell Voltages (Cells 25-36)",
    0xF302,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_37_48 = {
    "Opencell Voltages (Cells 37-48)",
    0xF303,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_49_60 = {
    "Opencell Voltages (Cells 49-60)",
    0xF304,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_61_72 = {
    "Opencell Voltages (Cells 61-72)",
    0xF305,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_73_84 = {
    "Opencell Voltages (Cells 73-84)",
    0xF306,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_85_96 = {
    "Opencell Voltages (Cells 85-96)",
    0xF307,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_97_108 = {
    "Opencell Voltages (Cells 97-108)",
    0xF308,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_109_120 = {
    "Opencell Voltages (Cells 109-120)",
    0xF309,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_121_132 = {
    "Opencell Voltages (Cells 121-132)",
    0xF30A,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_133_144 = {
    "Opencell Voltages (Cells 133-144)",
    0xF30B,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_145_156 = {
    "Opencell Voltages (Cells 145-156)",
    0xF30C,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_157_168 = {
    "Opencell Voltages (Cells 157-168)",
    0xF30D,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

const CanMessage opencell_voltages_169_180 = {
    "Opencell Voltages (Cells 169-180)",
    0xF23E,
    "Volts",
    "0",
    0,
    24,
    0.0f,
    5.0f,
    0.0001f,
    "Volts",
    "NOTE: Each message includes 12 voltages (each are 2 bytes long)"
};

// Internal resistance arrays (15 messages for cells 1-180)
const CanMessage internal_resistances_1_12 = {
    "Internal Resistances (Cells 1-12)",
    0xF200,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_13_24 = {
    "Internal Resistances (Cells 13-24)",
    0xF201,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_25_36 = {
    "Internal Resistances (Cells 25-36)",
    0xF202,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_37_48 = {
    "Internal Resistances (Cells 37-48)",
    0xF203,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_49_60 = {
    "Internal Resistances (Cells 49-60)",
    0xF204,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_61_72 = {
    "Internal Resistances (Cells 61-72)",
    0xF205,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_73_84 = {
    "Internal Resistances (Cells 73-84)",
    0xF206,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_85_96 = {
    "Internal Resistances (Cells 85-96)",
    0xF207,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_97_108 = {
    "Internal Resistances (Cells 97-108)",
    0xF208,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_109_120 = {
    "Internal Resistances (Cells 109-120)",
    0xF209,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_121_132 = {
    "Internal Resistances (Cells 121-132)",
    0xF20A,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_133_144 = {
    "Internal Resistances (Cells 133-144)",
    0xF20B,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_145_156 = {
    "Internal Resistances (Cells 145-156)",
    0xF20C,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_157_168 = {
    "Internal Resistances (Cells 157-168)",
    0xF20D,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};

const CanMessage internal_resistances_169_180 = {
    "Internal Resistances (Cells 169-180)",
    0xF20E,
    "mOhms",
    "0",
    0,
    24,
    0.0f,
    327.67f,
    0.01f,
    "mOhms",
    "NOTE: Bit 16 (the MSB) indicates whether the cell is actively balancing (1 = balancing, 0 = not balancing)."
};


float BDRCANLib::we_love_jaden_lee() {
    Serial.println("We love Jaden Lee!");
    return 42.0f; // The answer to life, the universe, and everything
}