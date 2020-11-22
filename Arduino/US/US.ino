/**
 *  Ioannis Broumas
 *  ioabro17@student.hh.se
 *  Nov 2020
**/

/**
   Hardware Connections:
    Arduino | HC-SR04
    -------------------
      5V    |   VCC
      7     |   Trig
      8     |   Echo
      GND   |   GND
*/
#include <RedBot.h>

// Speaker Pin
const int speaker = 9;

// Pins
const int TRIG_PIN = A1;
const int ECHO_PIN = A0;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

void setup() {
  // set the pin mode for the speaker to be an output
  pinMode(speaker, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  //Set Echo pin as input to measure the duration of 
  //pulses coming back from the distance sensor
  pinMode(ECHO_PIN, INPUT);

  // We'll use the serial monitor to view the sensor output
  Serial.begin(9600);
}

void loop() {

  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    Serial.println("Out of range");
  } else if ( cm < 17 ){
    Serial.println("Too close!");
    Serial.print(cm);
    Serial.print(" cm \t");
    Serial.print(inches);
    Serial.println(" in");
    tone(speaker, 2000); // The frequency value can be between 20-20000 Hz
  } else {
    Serial.print(cm);
    Serial.print(" cm \t");
    Serial.print(inches);
    Serial.println(" in");
  }

  // Wait at least 60ms before next measurement
  delay(60);
  noTone(speaker);
}
