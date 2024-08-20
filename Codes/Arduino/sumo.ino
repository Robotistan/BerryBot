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
#define TRIG_PIN 8
#define ECHO_PIN 9
#define MODE_BUTTON 10

#define TRACKER_THRESHOLD 1000
#define HighSpeed 255

// Variable
long duration;
int distance;
int leftSensor = 0;
int rightSensor = 0;
int counter = 0;
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

void hcsr() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH);

  distance = (duration / 2) / 29.1;

  //Serial.print(cm);
  //Serial.println("cm");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  attachMotor();
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MODE_BUTTON, OUTPUT);

  while (digitalRead(MODE_BUTTON)==0){
    Stop();
    Serial.println(digitalRead(MODE_BUTTON));
  }

}

void loop(){
  hcsr();
  leftSensor = analogRead(LEFT_SENSOR);
  rightSensor = analogRead(RIGHT_SENSOR);
  

  /*
  Serial.print("left:");
  Serial.println(leftSensor);
  Serial.print("  right:");
  Serial.println(rightSensor);
  */
  delay(20);
  if (distance <= 15) {
    if ((leftSensor >= TRACKER_THRESHOLD) || (rightSensor >= TRACKER_THRESHOLD)){
      Backward(HighSpeed);
      delay(100);
    }
    else if ((leftSensor < TRACKER_THRESHOLD) && (rightSensor < TRACKER_THRESHOLD)){
      Forward(HighSpeed);
      delay(100);
    }
    else
      Stop();
  }
  else{
    if ((leftSensor >= TRACKER_THRESHOLD) || (rightSensor >= TRACKER_THRESHOLD)){
        Backward(HighSpeed);
        delay(100);
    }
    else if ((leftSensor < TRACKER_THRESHOLD) && (rightSensor < TRACKER_THRESHOLD)){
        counter += 1;
        if (counter == 3){
          Forward(HighSpeed);
          delay(100);
          counter = 0;
        }
        else{
          Left(HighSpeed);
          delay(100);
          Stop();
        }
    }else
        Stop();
  }
}
