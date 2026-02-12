/*
    bdrcan.h - Library for bulldog racing CAN id protocol, as defined by 
    https://www.drivetraininnovation.com/file-share/62d657cb-4878-478e-a06c-e22040fd226f (can manual v2.4)
    
    */

    // Top-level CanMessage so .cpp can define globals easily

    #ifndef bdrcanlib_h
    #define bdrcanlib_h
    #include "Arduino.h"
    #include <ACAN_T4.h> // required

    struct CanMessage {
        const char* name;           // main name
        uint32_t id;                // CAN ID
        const char* alt;            // alternative name
        const char* byte;           // byte
        int bit_start;              // start bit
        int length;                 // length
        float min;                  // minimum value
        float max;                  // maximum value
        float scale;                // value multiplier
        const char* units;          // display units
        const char* description;    // a long string description
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

        // Create message structs
        messageStruct createMessageInv(uint32_t id, const uint8_t* data, uint8_t length);

        // Send OBD2 request for BMS
        void sendOBD2Request(uint16_t pid);

        // Get all CAN IDs
        static uint32_t* getAllCANIDs(int* count = nullptr);

        // Interpret messages - extract and scale values from raw CAN data
        float interpretInverterMessage(const messageStruct& msg, const CanMessage& definition);
        float interpretBMSMessage(const messageStruct& msg, const CanMessage& definition);
        
        // Find message definition by ID
        static const CanMessage* findMessageByID(uint32_t id);
        
        // Helper to determine message type
        static bool isInverterMessage(const CanMessage* msg);
        static bool isBMSMessage(const CanMessage* msg);

        static const int defmeslen = 8; // Standard CAN message size
        static const uint32_t OBD2_REQUEST_ID = 0x7DF; // Standard OBD2 request ID
        
    private:
        bool waitingForResponse = false;
    };
    

// Global CAN message descriptors (defined in bdrcanlib.cpp)
extern const CanMessage Set_AC_Current;
extern const CanMessage Set_Brake_Current;
extern const CanMessage Set_ERPM;
extern const CanMessage Set_Position;
extern const CanMessage Set_Relative_Current;
extern const CanMessage Set_Relative_Brake_Current;
extern const CanMessage Set_Digital_Output_1;
extern const CanMessage Set_Digital_Output_2;
extern const CanMessage Set_Digital_Output_3;
extern const CanMessage Set_Digital_Output_4;
extern const CanMessage Max_AC_Current;
extern const CanMessage Set_Maximum_AC_Brake_Current;
extern const CanMessage Max_DC_Current;
extern const CanMessage Set_Maximum_DC_Brake_Current;
extern const CanMessage Drive_Enable;

// Additional CAN messages
extern const CanMessage erpm;
extern const CanMessage duty_cycle;
extern const CanMessage input_voltage;
extern const CanMessage AC_current;
extern const CanMessage DC_current;
extern const CanMessage RESERVED_1;
extern const CanMessage controller_temperature;
extern const CanMessage motor_temperature;
extern const CanMessage fault_code;
extern const CanMessage RESERVED_2;
extern const CanMessage Id;
extern const CanMessage Iq;
extern const CanMessage throttle_signal;
extern const CanMessage brake_signal;
extern const CanMessage digital_input_1;
extern const CanMessage digital_input_2;
extern const CanMessage digital_input_3;
extern const CanMessage digital_input_4;
extern const CanMessage digital_input_1_2;
extern const CanMessage digital_input_2_2;
extern const CanMessage digital_input_3_2;
extern const CanMessage digital_input_4_2;
extern const CanMessage drive_enable;
extern const CanMessage capacitor_temp_limit;
extern const CanMessage DC_current_limit;
extern const CanMessage drive_enable_limit;
extern const CanMessage igbt_acceleration_temperature_limit;
extern const CanMessage igbt_temperature_limit;
extern const CanMessage input_voltage_limit;
extern const CanMessage motor_acceleration_temperature_limit;
extern const CanMessage motor_temperature_limit;
extern const CanMessage RPM_min_limit;
extern const CanMessage RPM_max_limit;
extern const CanMessage power_limit;
extern const CanMessage reserved_3;
extern const CanMessage reserved_4;
extern const CanMessage CAN_map_version;

// Orion BMS CAN messages
extern const CanMessage relays_status;
extern const CanMessage max_cells_supported_count;
extern const CanMessage populated_cell_count;
extern const CanMessage pack_charge_current_limit;
extern const CanMessage pack_discharge_current_limit;
extern const CanMessage signed_pack_current;
extern const CanMessage unsigned_pack_current;
extern const CanMessage pack_voltage;
extern const CanMessage pack_open_voltage;
extern const CanMessage pack_state_of_charge;
extern const CanMessage pack_amphours;
extern const CanMessage pack_resistance;
extern const CanMessage pack_depth_of_discharge;
extern const CanMessage pack_health;
extern const CanMessage pack_summed_voltage;
extern const CanMessage total_pack_cycles;
extern const CanMessage highest_pack_temperature;
extern const CanMessage lowest_pack_temperature;
extern const CanMessage avg_pack_temperature;
extern const CanMessage heatsink_temperature_sensor;
extern const CanMessage fan_speed;
extern const CanMessage requested_fan_speed;
extern const CanMessage low_cell_voltage;
extern const CanMessage low_cell_voltage_id;
extern const CanMessage high_cell_voltage;
extern const CanMessage high_cell_voltage_id;
extern const CanMessage avg_cell_voltage;
extern const CanMessage low_opencell_voltage;
extern const CanMessage low_opencell_voltage_id;
extern const CanMessage high_opencell_voltage;
extern const CanMessage high_opencell_voltage_id;
extern const CanMessage avg_opencell_voltage;
extern const CanMessage low_cell_resistance;
extern const CanMessage low_cell_resistance_id;
extern const CanMessage high_cell_resistance;
extern const CanMessage high_cell_resistance_id;
extern const CanMessage avg_cell_resistance;
extern const CanMessage input_power_supply_voltage;
extern const CanMessage fan_voltage;

// Cell voltage arrays (15 messages for cells 1-180)
extern const CanMessage cell_voltages_1_12;
extern const CanMessage cell_voltages_13_24;
extern const CanMessage cell_voltages_25_36;
extern const CanMessage cell_voltages_37_48;
extern const CanMessage cell_voltages_49_60;
extern const CanMessage cell_voltages_61_72;
extern const CanMessage cell_voltages_73_84;
extern const CanMessage cell_voltages_85_96;
extern const CanMessage cell_voltages_97_108;
extern const CanMessage cell_voltages_109_120;
extern const CanMessage cell_voltages_121_132;
extern const CanMessage cell_voltages_133_144;
extern const CanMessage cell_voltages_145_156;
extern const CanMessage cell_voltages_157_168;
extern const CanMessage cell_voltages_169_180;

// Opencell voltage arrays (15 messages for cells 1-180)
extern const CanMessage opencell_voltages_1_12;
extern const CanMessage opencell_voltages_13_24;
extern const CanMessage opencell_voltages_25_36;
extern const CanMessage opencell_voltages_37_48;
extern const CanMessage opencell_voltages_49_60;
extern const CanMessage opencell_voltages_61_72;
extern const CanMessage opencell_voltages_73_84;
extern const CanMessage opencell_voltages_85_96;
extern const CanMessage opencell_voltages_97_108;
extern const CanMessage opencell_voltages_109_120;
extern const CanMessage opencell_voltages_121_132;
extern const CanMessage opencell_voltages_133_144;
extern const CanMessage opencell_voltages_145_156;
extern const CanMessage opencell_voltages_157_168;
extern const CanMessage opencell_voltages_169_180;

// Internal resistance arrays (15 messages for cells 1-180)
extern const CanMessage internal_resistances_1_12;
extern const CanMessage internal_resistances_13_24;
extern const CanMessage internal_resistances_25_36;
extern const CanMessage internal_resistances_37_48;
extern const CanMessage internal_resistances_49_60;
extern const CanMessage internal_resistances_61_72;
extern const CanMessage internal_resistances_73_84;
extern const CanMessage internal_resistances_85_96;
extern const CanMessage internal_resistances_97_108;
extern const CanMessage internal_resistances_109_120;
extern const CanMessage internal_resistances_121_132;
extern const CanMessage internal_resistances_133_144;
extern const CanMessage internal_resistances_145_156;
extern const CanMessage internal_resistances_157_168;
extern const CanMessage internal_resistances_169_180;

    

#endif