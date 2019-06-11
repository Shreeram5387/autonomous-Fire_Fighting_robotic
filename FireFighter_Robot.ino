
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h> 

#define TRIG_PIN A0 
#define ECHO_PIN A1 
#define MAX_DISTANCE 300 
#define MAX_SPEED 190 // sets speed of DC  motors
#define MAX_SPEED_OFFSET 10

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 

AF_DCMotor right_motor(4, MOTOR12_8KHZ);
AF_DCMotor left_motor(3, MOTOR12_8KHZ);
AF_DCMotor three_motor(1, MOTOR12_8KHZ);

String readString;
Servo myservo;   

boolean goesForward=false;
int distance = 100;
int speedSet = 0;
void setup() {
  Serial.begin(9600);
  right_motor.setSpeed(255);
  left_motor.setSpeed(255);
  three_motor.setSpeed(255);
myservo.attach(9);  
  myservo.write(70); 
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  
  }


void loop() 
{
  while(Serial.available()){
    delay(50);
    char c=Serial.read();
    readString+=c;
  }
  if(readString.length()>0){
    Serial.println(readString);
    if (readString =="FORWARD"){ //forward
      right_motor.run (FORWARD);
      left_motor.run (FORWARD);
      delay(500);
    }
    if (readString =="BACKWARD"){ //backward
      right_motor.run (BACKWARD);
      left_motor.run (BACKWARD);
      delay(500);
    }
    if (readString =="LEFT"){ //left
      right_motor.run (FORWARD);
      left_motor.run (BACKWARD);
      delay(500);
    }
    if (readString =="RIGHT"){  //right
      right_motor.run (BACKWARD);
      left_motor.run (FORWARD);
      delay(500);
    }
    if (readString =="STOP"){  //stop
      right_motor.run (RELEASE);
      left_motor.run (RELEASE);
      delay(500);
    }
    if (readString =="RUN WATER"){  //ON Water Pump
      three_motor.run (FORWARD);
      delay(500);
    }
    if (readString =="STOP WATER"){  //OFF Water Pump
      three_motor.run (RELEASE);
      delay(500);
    }
    if (readString =="AUTO"){  //OFF Water Pump
int distanceR = 0;
 int distanceL =  0;
 delay(40);
 
 if(distance<=50)
 {
  moveStop();
  delay(100);
  moveBackward();
  delay(500);
  moveStop();
  delay(200);

  distanceR = lookRight();
  delay(200);
  distanceL = lookLeft();
  delay(200);

  if(distanceR>=distanceL)
  {
    turnRight();
    moveStop();
  }else
  {
    turnLeft();
    moveStop();
  }
 }else
 {
  moveForward();
  myservo.write(70); 

 }
 distance = readPing();
      delay(500);
  
  readString="";
  }
 
}
}
int lookRight()
{
    myservo.write(30); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(80); 
    delay(200);

    return distance;
    
}

int lookLeft()
{
    myservo.write(80); 
    delay(500);
    int distance = readPing();
    delay(100);
    myservo.write(130); 
      delay(200);
    return distance;
}

int readPing() { 
  delay(70);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  return cm;
}

void moveStop() {
  left_motor.run(RELEASE); 
  right_motor.run(RELEASE);
  } 
  
void moveForward() {

 if(!goesForward)
  {
    goesForward=true;
    left_motor.run(FORWARD);      
    right_motor.run(FORWARD); 
   for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
   {
    left_motor.setSpeed(speedSet);
    right_motor.setSpeed(speedSet+MAX_SPEED_OFFSET);
    delay(5);
 
   }
  }
}

void moveBackward() {
    goesForward=false;
    left_motor.run(BACKWARD);      
    right_motor.run(BACKWARD);  
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    left_motor.setSpeed(speedSet);
    right_motor.setSpeed(speedSet+MAX_SPEED_OFFSET);
    delay(5);
  
  }
}  

void turnRight() {
  left_motor.run(FORWARD);
  right_motor.run(BACKWARD);     
  delay(1000);

//  motor1.run(FORWARD);      
//  motor2.run(FORWARD);      
} 
 
void turnLeft() {
  left_motor.run(BACKWARD);     
  right_motor.run(FORWARD);     
  delay(1000);
  
//  motor1.run(FORWARD);     
//  motor2.run(FORWARD);
}  


