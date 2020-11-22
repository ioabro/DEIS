/***********************************************************************
 * Ioannis Broumas
 * ioabro17@student.hh.se
 * Nov 2020
 * Serial drive 
***********************************************************************/

#include <RedBot.h>

RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10);  // initializes encoder on pins A2 and 10
RedBotAccel accelerometer;

int leftPower;  // variable for setting the drive power
int rightPower;
int data;  // variable for holding incoming data from PC to Arduino

int countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

// variables used to store the left and right encoder counts.
int lCount;
int rCount;

void setup(void)
{
    Serial.begin(115200); 
    // Serial.print("Enter in left and right motor power values and click [Send]."); 
    // Serial.print("Separate values with a space or non-numeric character.");
    // Serial.println();
    // Serial.print("Positive values spin the motor CW, and negative values spin the motor CCW.");
}

void loop(void)
{
    encoder.clearEnc(BOTH);  // Reset the counters.

    // if there is data coming in on the Serial monitor, do something with it.
    if(Serial.available() > 0)
    {
        leftPower = Serial.parseInt();  // read in the next numeric value
        leftPower = constrain(leftPower, -255, 255);  // constrain the data to -255 to +255
        
        rightPower = Serial.parseInt();   // read in the next numeric value
        rightPower = constrain(rightPower, -255, 255);  // constrain the data to -255 to +255

        motors.leftMotor(leftPower);
        motors.rightMotor(rightPower);

        // short delay in between readings/
        delay(100); 

        // store the encoder counts to a variable.
        lCount = encoder.getTicks(LEFT);    // read the left motor encoder
        rCount = encoder.getTicks(RIGHT);   // read the right motor encoder

        Serial.print(lCount);  // tab
        Serial.print("_");  // tab
        Serial.print(rCount);  // tab
        Serial.print(" ");  // tab

        accelerometer.read(); // updates the x, y, and z axis readings on the acceleromter

        // Display out the X, Y, and Z - axis "acceleration" measurements and also
        // the relative angle between the X-Z, Y-Z, and X-Y vectors. (These give us 
        // the orientation of the RedBot in 3D space. 

        Serial.print(accelerometer.x);
        Serial.print("_");  // tab

        Serial.print(accelerometer.y);
        Serial.print("_");  // tab

        Serial.print(accelerometer.z);
        Serial.print(" ");  // tab

        Serial.print(accelerometer.angleXZ);
        Serial.print("_"); 
        Serial.print(accelerometer.angleYZ);
        Serial.print("_"); 
        Serial.print(accelerometer.angleXY);
        Serial.print(" ");
    } 
    else
    {
        motors.brake();
    }
     
}