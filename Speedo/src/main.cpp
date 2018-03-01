#include <Arduino.h>
#include <FreqMeasure.h>
#include <SwitecX25.h>
#include <Wire.h>
#include <MovingAverageFilter.h>
MovingAverageFilter movingAverageFilter(20);

//#define MAXSTEP 738     // 0-6000rpm
#define MAXSTEP 692       // 0-85mph 0-137kmh
const float kmhXToSteps = 5.0;  //Multiply for kmh to gauge steps

#define STEPS (315*3)   // standard X25.168 range 315 degrees at 1/3 degree steps
#define AVRGCOUNT 6     // How many speed Hz reading using for average value
double sum=0;           // Memory for average value sum
int count=0;            // Memory for average how many values in memory

float frequency = 0.0;      // Frequency as Speed
float frequencyFilt = 0.0;  // Frequency as Speed
const float freqXToSpeed = 1.0; // Kerroin millä muuttaa taajuus nopeudeksi km/h
float fSpeedKmh = 0.0;       // Speed as km/h
unsigned char iSpeed = 0;   // speed as integer 8bit

// Distance
unsigned int iOdoPulsesIn = 0;      // Memory for odometer input pulses
const int pulsesPStep = 100;        //How many pulses is one odometer step
const int odoPulseInPIN = 7;        // Input pin for odometer pulses
const int odoStepOutPIN = 8;        //Output pin for pulse output for odometer stepper
const int odoStepEnableOutPIN = 9;  // Output for enable odometer stepper

// For motors connected to digital pins 4,5,6,7
//SwitecX25 motor1(STEPS,9,8,6,7); RPM
SwitecX25 motor1(STEPS,16,10,15,14);  // Speedo gauge
int iPosition = 0;                       // Setpoint to motor (speed)
unsigned long lastMeasureTime = 0;    // millis when lasti speed measure has taken
const unsigned long zeroTimeOut = 3000; // ms, no values set speed zero
unsigned long lastSteppedTime = 0;      // millis when last odo step has occur
const unsigned long StepEnableTimeOut = 10000; // ms, no steps, enable off timer

unsigned long lastMillis = 0;


void requestEvent() {   // I2C reply event
  Wire.write(iSpeed);   // Send speed to master if it ask.
}

void odocounter()  {      // interrupt to count pulses for odometer
  iOdoPulsesIn = iOdoPulsesIn +1;
}

void setup() {
    // put your setup code here, to run once:
    //Serial.begin(115200);       // For debuggin
    delay(50);            // Wait for it everyting is started and voltage rised
    motor1.zero();        // Reset gauge to zero
    delay(200);           // Wait to gauge calm down
    pinMode(odoPulseInPIN , INPUT);    // Pin 7 Odometer counter
    FreqMeasure.begin();  // Pin 4 speedo input
    delay(100);
    Wire.begin(2);                // join i2c bus with address #2 speedo (1# RPM)
    Wire.onRequest(requestEvent); // register event
    attachInterrupt(digitalPinToInterrupt(odoPulseInPIN), odocounter, FALLING);  // Pin 7 Odometer counter
    pinMode(odoStepOutPIN, OUTPUT);   // odometer stepper pulses output
    pinMode(odoStepEnableOutPIN, OUTPUT);   // odometer stepper enable output
    digitalWrite(odoStepEnableOutPIN, HIGH);  // Enable LOW active, HIGH SLEEP
}

void loop() {
    // put your main code here, to run repeatedly:
    lastMillis = millis();
    //Serial.println(iOdoPulsesIn);
    motor1.update();

    if (FreqMeasure.available()) {
      // average several reading together
      sum = sum + FreqMeasure.read();
      count = count + 1;
      if (count > AVRGCOUNT) {
        frequency = FreqMeasure.countToFrequency(sum / count);
        sum = 0;
        count = 0;
        lastMeasureTime = lastMillis;       // Set this time to memory
        frequency = frequency / 2;
        frequencyFilt = movingAverageFilter.process(frequency);
        fSpeedKmh = frequencyFilt * freqXToSpeed;  // Multiply frequency to speed
        iPosition = int(fSpeedKmh * kmhXToSteps);  // Calculate position for speedo
        iSpeed = char(fSpeedKmh);                  //Get int from float for speed for I2C
        if(iPosition >= MAXSTEP) {            // Check is steps in range
          iPosition = MAXSTEP;
        }
        else if (iPosition <= 0) {
          iPosition = 0;
        }
        // else iPosition = iPosition;

      }
    }
    /*
    Serial.print("F");
    Serial.print(frequency);
    Serial.print(" K");
    Serial.print(fSpeedKmh);
    Serial.print(" S");
    Serial.println(iPosition);
    */

    if (iOdoPulsesIn >= pulsesPStep) {    //  Is time to move odometer?
      // generate step for odometer stepper
      digitalWrite(odoStepEnableOutPIN, LOW);   //Enable low active
      delayMicroseconds(2);
      digitalWrite(odoStepOutPIN, HIGH);
      delayMicroseconds(1);       // = About 3-4uS pulse
      digitalWrite(odoStepOutPIN, LOW);
      iOdoPulsesIn = iOdoPulsesIn - pulsesPStep;  // subtract step in counter
      lastSteppedTime = lastMillis;                // set this time to memory
    }

    if(lastMillis - lastSteppedTime >= StepEnableTimeOut)   // Enable off timer
    {
      digitalWrite(odoStepEnableOutPIN, HIGH);              // LOW is active
    }

    if(lastMillis - lastMeasureTime >= zeroTimeOut)   // No pulses set all to zero
    {
      frequency = 0;
      fSpeedKmh = 0;
      iPosition = 0;
      iSpeed = 0;
      //digitalWrite(odoStepEnableOutPIN, LOW);
    }
    //Serial.println(iOdoPulsesIn);
    motor1.setPosition(iPosition);
}
