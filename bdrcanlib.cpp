#include "Arduino.h"
#include "bdrcanlib.h"

bdrcanlib::bdrcanlib() {
    id = 0;
    length = 0;
    memset(data, 0, sizeof(data));
}

CanMessage bdrcanlib::createmessage() {
    return CanMessage{id, data, length};
}

/*
 * Define every CAN ID used in the system.
 * Add or modify as needed for your application.
 */
const uint32_t CAN_ID_ENGINE_STATUS   = 0x100;
const uint32_t CAN_ID_VEHICLE_SPEED  = 0x101;
const uint32_t CAN_ID_BRAKE_STATUS   = 0x102;
const uint32_t CAN_ID_STEERING_ANGLE = 0x103;
const uint32_t CAN_ID_FUEL_LEVEL     = 0x104;
const uint32_t CAN_ID_DOOR_STATUS    = 0x105;
const uint32_t CAN_ID_LIGHT_STATUS   = 0x106;
const uint32_t CAN_ID_AIRBAG_STATUS  = 0x107;
const uint32_t CAN_ID_BATTERY_VOLTAGE= 0x108;
const uint32_t CAN_ID_ODOMETER       = 0x109;

