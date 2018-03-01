#include <Arduino.h>
#include <Wire.h>
#include <SwitecX25.h>

// standard X25.168 range 315 degrees at 1/3 degree steps
#define MAXSTEP 738 // 0-6000rpm
//#define MAXSTEP 692 // 0-85mph
#define STEPS (315*3)

SwitecX25 motor1(STEPS,9,8,6,7);  //RPM For motors connected
//SwitecX25 motor1(STEPS,16,10,15,14); // Speedo
unsigned int iPosition = 0;
unsigned int iInputData = 0;



void receiveEvent(int howMany) {
  //Serial.println(howMany);
  if (2 <= howMany) {             // if two bytes were received
      iInputData = Wire.read();   // receive high byte (overwrites previous reading)
      iInputData = iInputData << 8; // shift high byte to be high 8 bits
      iInputData |= Wire.read();  // receive low byte as lower 8 bits
      if (iInputData <= 0 )  {    // Check is readings ok?
        iPosition = 0;
      }
      else if (iInputData > MAXSTEP) {
        iPosition = MAXSTEP;
      }
      else  {
        iPosition = iInputData;
      }
      //Serial.println(reading);
    }
}

void setup() {
    // Setup
    //Serial.begin(115200);
    delay(50);                   // Wait for it everyting is started and voltage rised
    motor1.zero();                // Reset gauge to zero
    delay(200);                   // Wait to gauge calm down
    Wire.begin(1);                // join i2c bus with address #1
    Wire.onReceive(receiveEvent); // register event
}

void loop() {
    // Main
    motor1.update();                // Update gauge move
    motor1.setPosition(iPosition);  // Update gauge target position
}
