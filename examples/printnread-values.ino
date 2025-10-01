// this code will print everything to serial in arduino IDE. 
// it will print everything requested

#include <Arduino.h>

#include <bdrcanlib.h>

#include <SPI.h>
#include <mcp2515.h> // can read library

struct can_frame canMsg;
MCP2515 mcp2515(10);

CanMessage[] mess = {Set_AC_Current, Set_Brake_Current};

void setup() {
  Serial.begin(115200);
    while (!Serial) { ; } // wait for serial port to connect. Needed for native USB
    delay(2000); // wait for serial to initialize

    Serial.println("Starting up...");
    
    for (int i = 0; i < sizeof(mess); i++) {
        Serial.println(mess[i].name);
        Serial.println(mess[i].description);
        Serial.println("ID: " + String(mess[i].id, HEX));
        Serial.println("Unit: " + String(mess[i].unit));
        Serial.println("Min: " + String(mess[i].min));
        Serial.println("Max: " + String(mess[i].max));
        Serial.println("Scale: " + String(mess[i].scale));
        Serial.println("Byte: " + String(mess[i].byte));
        Serial.println("Bit start: " + String(mess[i].bit_start));
        Serial.println("Length: " + String(mess[i].length));
        Serial.println(); // Print a blank line between messages for readability
        
    }
    Serial.println("beginning CAN communication...");
    Serial.println("ID  DLC   DATA");
    
}

void loop(){
    // nothing here
    read();
    unsigned long id = canMsg.can_id;

}


void canMsg read() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();
  }
}
