This repository provides CAN ID definitions for the Yale Bulldog Racing team, intended for use as a Teensy 4.1 library.

# bdr-can-ids repository manual 

## About this library 

This is the documentation for the Github repository called bdr-can-ids. It enables easy access to the motor inverter’s can id communication protocol, which is proprietary, and thus, arbitrary. Thus, ease of access is achieved through the existence of this github repository, in the form of an Arduino library. 

## Who should use this library? 

People who are using Teensy/other Arduino-based systems can use this library.  

## Hardware requirements 

- Teensy 4.01 

### recognized hardware:

- Motor Inverter (from DTI) 

- orion2 bms

## Quickstart

1. Navigate to the repository page on Github bdr-can-ids 

2. Download as .zip file 

3. Open Arduino IDE 

4. Add library as zip (on MacOS sketch -> include library -> add .ZIP library…) 

## usage

### structs
each id is given a CanMessage construct, to store the kind of message each id is. it looks like this:
```cpp
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
```
here additionally is the message struct. it contains a can message
```cpp
struct messageStruct {
        uint32_t id;
        uint8_t data[8];
        uint8_t length;
    };
```
### functions

there are several basic functions this library has:

#### important functions 
**createMessageInv**
```cpp
messageStruct createMessageInv(uint32_t id, const uint8_t* data, uint8_t length);
```
creates an inverter message.

**sendOBD2Request**
```cpp
void sendOBD2Request(uint16_t pid);
```
sends an OBD2 request. the orion2 bms uses a specific type of can, and waits for a message request before sending any kind of can message, unless manually configured otherwise

#### other functions

**getAllCANIDs** 
```cpp 
static uint32_t* getAllCANIDs();
```
gets an array of every id (for display purposes)

**conv_to_dec** 
```cpp 
static float conv_to_dec(const String& s);
```
converts a string number with decimals separated by commas into floats separated by periods, ie "1,32" -> 1.32
In the motor inverter documentation, it is common to find these kinds of numbers.


### ids:
here is a list of recognised id messages that this library can use:

#### dti motor inverter: 
- Set_AC_Current
- Set_Brake_Current
- Set_ERPM
- Set_Position
- Set_Relative_Current
- Set_Relative_Brake_Current
- Set_Digital_Output_1
- Set_Digital_Output_2
- Set_Digital_Output_3
- Set_Digital_Output_4
- Max_AC_Current
- Set_Maximum_AC_Brake_Current
- Max_DC_Current
- Set_Maximum_DC_Brake_Current
- Drive_Enable

#### orion2 bms

**General BMS Messages:**
- relays_status
- max_cells_supported_count
- populated_cell_count
- pack_charge_current_limit
- pack_discharge_current_limit
- signed_pack_current
- unsigned_pack_current
- pack_voltage
- pack_open_voltage
- pack_state_of_charge
- pack_amphours
- pack_resistance
- pack_depth_of_discharge
- pack_health
- pack_summed_voltage
- total_pack_cycles
- highest_pack_temperature
- lowest_pack_temperature
- avg_pack_temperature
- heatsink_temperature_sensor
- fan_speed
- requested_fan_speed
- low_cell_voltage
- low_cell_voltage_id
- high_cell_voltage
- high_cell_voltage_id
- avg_cell_voltage
- low_opencell_voltage
- low_opencell_voltage_id
- high_opencell_voltage
- high_opencell_voltage_id
- avg_opencell_voltage
- low_cell_resistance
- low_cell_resistance_id
- high_cell_resistance
- high_cell_resistance_id
- avg_cell_resistance
- input_power_supply_voltage
- fan_voltage

**Cell Voltage Arrays (15 messages for cells 1-180):**
- cell_voltages_1_12
- cell_voltages_13_24
- cell_voltages_25_36
- cell_voltages_37_48
- cell_voltages_49_60
- cell_voltages_61_72
- cell_voltages_73_84
- cell_voltages_85_96
- cell_voltages_97_108
- cell_voltages_109_120
- cell_voltages_121_132
- cell_voltages_133_144
- cell_voltages_145_156
- cell_voltages_157_168
- cell_voltages_169_180

**Opencell Voltage Arrays (15 messages for cells 1-180):**
- opencell_voltages_1_12
- opencell_voltages_13_24
- opencell_voltages_25_36
- opencell_voltages_37_48
- opencell_voltages_49_60
- opencell_voltages_61_72
- opencell_voltages_73_84
- opencell_voltages_85_96
- opencell_voltages_97_108
- opencell_voltages_109_120
- opencell_voltages_121_132
- opencell_voltages_133_144
- opencell_voltages_145_156
- opencell_voltages_157_168
- opencell_voltages_169_180

**Internal Resistance Arrays (15 messages for cells 1-180):**
- internal_resistances_1_12
- internal_resistances_13_24
- internal_resistances_25_36
- internal_resistances_37_48
- internal_resistances_49_60
- internal_resistances_61_72
- internal_resistances_73_84
- internal_resistances_85_96
- internal_resistances_97_108
- internal_resistances_109_120
- internal_resistances_121_132
- internal_resistances_133_144
- internal_resistances_145_156
- internal_resistances_157_168
- internal_resistances_169_180


## Source 

DTI motor inverter can manual 

All the IDs and other related information were taken from this manual, compiled in a spreadsheet, exported as CSV, then auto-formatted (through a Python script of my own design) into a library. 

---

**License**

MIT License

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
