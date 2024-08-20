// Libraries
#include <Wire.h>
#include <stdio.h>

// Pin Defination
#define MOTOR_B1 22
#define MOTOR_B2 23
#define MOTOR_A2 24
#define MOTOR_A1 25
#define PWM_A 15
#define PWM_B 21
#define LEFT_SENSOR 26
#define RIGHT_SENSOR 27
#define MODE_BUTTON 10

#define TRACKER_THRESHOLD 900

#define STOP  0
#define FWD   1
#define BWD   2
#define RIGHT 3
#define LEFT  4

// Variable
int leftSensor = 0;
int rightSensor = 0;
uint8_t directionStt = STOP;
uint8_t oldDirection = STOP;
unsigned long reverseTime = 0;

void attachMotor()
{
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
}

void Forward(int speed)
{
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

void Backward(int speed)
{
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

void Left(int speed)
{
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

void Right(int speed)
{
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

void Stop()
{
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  attachMotor();
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(MODE_BUTTON, OUTPUT);

  while (digitalRead(MODE_BUTTON)==0){
    Stop();
    Serial.println(digitalRead(MODE_BUTTON));
  }

}

void loop(){
  leftSensor = analogRead(LEFT_SENSOR);
  rightSensor = analogRead(RIGHT_SENSOR);
  
  /*
  Serial.print("left:");
  Serial.println(leftSensor);
  Serial.print("  right:");
  Serial.println(rightSensor);
  */

  if (leftSensor >= TRACKER_THRESHOLD && rightSensor >= TRACKER_THRESHOLD) {
    directionStt = FWD;
    //Serial.println("FWD");
  } 
  else if (leftSensor < TRACKER_THRESHOLD && rightSensor > TRACKER_THRESHOLD) {
    directionStt = RIGHT;
    //Serial.println("RIGHT");
  } 
  else if (leftSensor > TRACKER_THRESHOLD && rightSensor < TRACKER_THRESHOLD) {
    directionStt = LEFT;
    //Serial.println("LEFT");
  } 
  else if (leftSensor < TRACKER_THRESHOLD && rightSensor < TRACKER_THRESHOLD && directionStt != STOP) {
    directionStt = BWD;
    //Serial.println("BWD");
  }

  if (directionStt != oldDirection) {
    oldDirection = directionStt;
    if (directionStt == FWD)
      Forward(220);
    else if (directionStt == RIGHT)
      Right(220);
    else if (directionStt == LEFT)
      Left(220);
    else if (directionStt == BWD) {
      //Backward(140);
      reverseTime = millis();
    }
    /*else if (directionStt == STOP)
      Stop();
    */
  }

  if (directionStt == BWD && millis() - reverseTime > 300)
    directionStt = STOP;

}
