#include <Servo.h> //include Servo library
#include<SoftwareSerial.h>
#include <AFMotor.h>
#include <NewPing.h>
#define MAX_DISTANCE 200 

#define TRIG_PIN A0   // HC_SR04 trig pin to Arduino pin 3
#define ECHO_PIN A1   // HC_SR04 echo pin to Arduino pin 2 (this pin is Arduino's interrupt pin 0)

AF_DCMotor motor1(3); //left
AF_DCMotor motor2(4);  //right



int sensorPinforwardd = A3; // select the input pin for the LDR
int sensorPinleftt = A4; // select the input pin for the LDR
int sensorPinrightt = A5; // select the input pin for the LDR
int sensorValueforward = 0;
int sensorValueleft = 0;
int sensorValueright= 0;
int led = 7 ;// Output pin for LED
int buzzer = 5; // Output pin for Buzzer


const int collisionThresh   = 50; //threshold for obstacles (in cm)
                                  // the sensors to the light source. NOTE: this is ANALOG pin 4
int leftDistance, rightDistance; //distances on either side

Servo panMotor;   //The micro servo with the distance sensor on it
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
int distance = 0;


void setup()
{
 pinMode(led, OUTPUT);

pinMode(buzzer,OUTPUT);
  panMotor.attach(9);     // attache the micro servo
  panMotor.write(70);     //center the micro servo

  Serial.begin(9600);
  Serial.println("This monitor will show us where the robot will go");
int distance = readPing();  //c/call the ping function to get the distance in front of the robot
delay(200);

int distance = readPing();  //c/call the ping function to get the distance in front of the robot
delay(200);

int distance = readPing();  //c/call the ping function to get the distance in front of the robot
delay(200);
Serial.print("cm:");
Serial.println(distance)
}

void loop(){

 int threshold = 100;
sensorValueleft = analogRead(sensorPinleftt);  //read the left light sensor
Serial.print("Left:  ");
Serial.println(sensorValueleft);
sensorValueright = analogRead(sensorPinrightt);
Serial.print("Right:  ");
Serial.print(sensorValueright);
sensorValueforward = analogRead(sensorPinforwardd);
Serial.print("Straight:");
Serial.println(sensorValueforward);
delay(200);
 int distance = readPing();  //c/call the ping function to get the distance in front of the robot
delay(200);
Serial.print("cm:");
Serial.println(distance);

  if (distance > collisionThresh) //if path is clear, the robot can use the light sensors to navigate towards the light
  {
  // Check the conditions for moving forward
    if ( sensorValueleft       > threshold     && 
       sensorValueright      > threshold     && 
       sensorValueforward    > threshold        ) 
       {  
          Serial.print(" GOO Forward");
          motor1.run(FORWARD);     
          motor2.run(FORWARD);
          delay(300);
        }

  //This condition indicates that the robot has reached the light source,
  //so it will stop
  else if ( sensorValueforward    < threshold/2 && 
         sensorValueforward   <   threshold/7    ) 
       {  
          Serial.println("STOP");
            motor1.run(RELEASE); 
            motor2.run(RELEASE); 
          
          delay(500);
        }
        
  // If the left light sensor value is smaller than that of the right sensor, 
  // and the difference between the values is large, then
  // the light source is towards the left, so robot should turn left. 
  else if ( sensorValueleft       <   threshold &&
            sensorValueright       >     threshold    ) 
            { 
              Serial.print("LEFT");
                motor1.run(BACKWARD);     
                motor2.run(FORWARD);     
                delay(80);
             
            }

  // If the right light sensor value is smaller than that of the left sensor, 
  // and the difference between the values is large, then
  // the light source is towards the right, so robot should turn right. 
  else if ( sensorValueright       <  threshold  &&        
            sensorValueleft       > threshold ) 
            { 
                Serial.print("RIGHT");
                 motor1.run(FORWARD);     
                motor2.run(BACKWARD);     
                delay(80);
            }

  
  }
  else    // The path is blocked, so the robot will use the distance sensor to find a clear path
  {
      Serial.print(" blocked ");
      motor1.run(RELEASE);     
      motor2.run(RELEASE);       // Stop the right motor
      panMotor.write(0); 
      delay(500);
      rightDistance = readPing();  //scan to the right rightDistance
      delay(300);
      panMotor.write(150);
      delay(500);
      leftDistance = readPing(); //scan to the left
      delay(300);
      panMotor.write(70);     //return to center
      delay(100);
     compareDistance();
  }
  

}

void compareDistance()
{
  if (leftDistance > rightDistance) //if left is less obstructed 
  {
     motor1.run(BACKWARD);     
     motor2.run(FORWARD);
    Serial.print("Left");
    delay(400); 
    motor1.run(FORWARD);     
     motor2.run(FORWARD);
    delay(600);
  }
  else if (rightDistance > leftDistance) //if right is less obstructed
  {   
     motor1.run(FORWARD);     
     motor2.run(BACKWARD);
    Serial.print("Right");
    delay(400);
     motor1.run(FORWARD);     
     motor2.run(FORWARD);
    delay(600);
  }
   else //if they are equally obstructed
  {
     motor1.run(BACKWARD);     
     motor2.run(FORWARD);
    Serial.print("Back");
    delay(400);
  }
}

int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
}
