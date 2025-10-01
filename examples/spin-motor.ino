#include <FlexCAN_T4.h>

#include <bdrcanlib.h>

#include <Arduino.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

// Modifed Hezheng Code for PedalBox Node
// CANH protocol sheet 
// https://docs.google.com/spreadsheets/d/1TxSrHOldQo2aFybV2Fm9HpDr5VunyQKS_ToyPHl7ahg/edit?usp=sharing




#define AVG(a, b) ((a + b) / 2)


#define enable_drive_ON 0x01                // payload to enable drive
#define enable_drive_LEN 1                  // length of drive enable msg

#define SET_CURRENT_LEN 2                   // length of set current msg

#define CURRENT_MAX 2000                    // max current in jApk

#define ACCEPTABLE_DIFF 400                 // diff
#define ITERATION_TIME 10                   // send msg every 10 ms

// define pins
#define POT_PIN_ONE 19                      //redifine pins
#define POT_PIN_TWO 233                     //redifine pins
#define READY_MSG 0x07FE                    // id for ready to drive msg (NOT INCLUDED IN PROTOCOL DOC)

#define SET_CURRENT_ID 0x011F
#define SET_CURRENT_LEN 2

void enable_drive();
bool ready = false;

void setup(void) {
    Serial.begin(115200); delay(400);
    Can0.begin();
    Can0.setBaudRate(500000);               // bitrate
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.mailboxStatus();
    Can0.onReceive(ready_to_drive);

    while (!ready) Can0.events();
    Can0.disableFIFOInterrupt();
    Can0.onReceive(NULL);
    enable_drive();
}

void ready_to_drive(const CAN_message_t &msg) {
    if (msg.id == READY_MSG)
        ready = true;
}


void enable_drive(){
    CAN_message_t msg;
    msg.id = Drive_Enable.id;
    msg.len = Drive_Enable.length; 
    for (uint8_t j = 0; j < msg.len; j++ ) msg.buf[j] = enable_drive_ON; // set the payload to the length
    Can0.write(msg);
}

void set_current(int16_t amps) {
    // amps: signed value in A_pk
    int16_t scaled = amps * 10;   // scale according to DBC

    CAN_message_t msg;
    msg.id = Set_AC_Current.id;   // from your bdrcanlib
    msg.len = Set_AC_Current.length;

    msg.buf[0] = scaled & 0xFF;         // LSB
    msg.buf[1] = (scaled >> 8) & 0xFF;  // MSB

    Can0.write(msg);
}


uint8_t curr_val[SET_CURRENT_LEN];


void loop() {
    long long start_time = millis();

    set_current(150);  // request 150 A

    long long end_time = millis();
    if ((end_time - start_time) < ITERATION_TIME) 
        delay(ITERATION_TIME - (end_time - start_time));
}

