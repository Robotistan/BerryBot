// Libraries
#include <Wire.h>
#include <stdio.h>

#define LDR_THRESHOLD 160
#define LDR_TOLERANCE 190

// Pin Defination
#define MOTOR_B1 22
#define MOTOR_B2 23
#define MOTOR_A2 24
#define MOTOR_A1 25
#define PWM_A 15
#define PWM_B 21
#define LDR_R_PIN 28
#define LDR_L_PIN 29
#define TRIG_PIN 8
#define ECHO_PIN 9
#define MODE_BUTTON 10

// Variable
long duration;
int distance;
float LDR_L;
float LDR_R;

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
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

void Right(int speed)
{
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
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
  pinMode(LDR_L_PIN, INPUT_PULLUP);
  pinMode(LDR_R_PIN, INPUT_PULLUP);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MODE_BUTTON, OUTPUT);

  while (digitalRead(MODE_BUTTON)==0){
    Stop();
  }

}

void loop(){
  hcsr();
  delay(2);
  LDR_L = analogRead(LDR_L_PIN);
  //Serial.print("LDR_L : ");
  //Serial.println(LDR_L);

  LDR_R = analogRead(LDR_R_PIN);
  //Serial.print("LDR_R : ");
  //Serial.println(LDR_R);
  

  if((LDR_L >= LDR_THRESHOLD) && (LDR_R >= LDR_THRESHOLD)){
    if (distance < 10) {
      Stop();
      delay(2);
      Left(150);
      delay(500);
      Stop();
    }
    else if((LDR_R - LDR_L) >= LDR_TOLERANCE){
      Right(150);
    }
    else if((LDR_L - LDR_R) >= LDR_TOLERANCE){
      Left(150);
    }
    else if((LDR_L >= 110) && (LDR_R >= 110)){
      Forward(255);
    }
    else{
      Stop();
    }
  }
  else{
    Stop();
  }

}
