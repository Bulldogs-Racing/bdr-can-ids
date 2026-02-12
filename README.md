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
