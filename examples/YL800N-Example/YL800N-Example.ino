#define _YL800N_DEBUG_

#include <SoftwareSerial.h>
#include <YL-800N.h>

SoftwareSerial mySerial(10, 11);
YL800N lora(mySerial);
long nextMillis = 5000;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting YL800N");

  mySerial.begin(9600);

  lora.setConfig(
    FRAME_MODULE_CONFIG::CHANNEL::CH432M,
    FRAME_MODULE_CONFIG::USER_MODE::HEXADECIMAL,
    FRAME_MODULE_CONFIG::ROLE::SLAVE,
    0x0000,  // network_flag
    1);      // node_flag
}

int i = 0;

void loop() {
  if (millis() >= nextMillis) {
    String msg = "MSG ";
    msg.concat(i++);    
    lora.println(msg);
    
    String line;
    if(lora.readLine(line)) {
      Serial.print("TEXT: ");
      Serial.println(line);      
    }

    nextMillis += 5000;
  }
}
