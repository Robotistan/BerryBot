// Libraries
#include <Wire.h>
#include <stdio.h>
#include "RPi_Pico_TimerInterrupt.h"

#define LDR_THRESHOLD 250
#define LDR_TOLERANCE 75

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

// Led Matrix
int sunny[5] =     {0x15,0x0E,0x1F,0x0E,0x15};

// Variable
long duration;
int distance;
float LDR_L;
float LDR_R;

int rowPins[5] = {7, 11, 12, 13, 17}; //Row LedMatrix Pins
int colPins[5] = {18, 19, 16, 2, 3};  //Col LedMatrix Pins

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

volatile int ledRow = 0;
volatile int prevLedRow = 4;
volatile byte ledArrayBuffer[5];

void drawScreen(int buffer[]){
  for (int i = 0; i < 5; i++) {
      ledArrayBuffer[i] = buffer[i];
  }
}

void setColumns(byte b) {
    digitalWrite(colPins[0], (~b >> 0) & 0x01); 
    digitalWrite(colPins[1], (~b >> 1) & 0x01); 
    digitalWrite(colPins[2], (~b >> 2) & 0x01); 
    digitalWrite(colPins[3], (~b >> 3) & 0x01); 
    digitalWrite(colPins[4], (~b >> 4) & 0x01); 
}

bool TimerHandler0(struct repeating_timer *t)
{
  (void) t; 
  if (ledRow == 5)
    ledRow = 0;
  setColumns(ledArrayBuffer[ledRow]); 
  digitalWrite(rowPins[ledRow], HIGH);
  digitalWrite(rowPins[prevLedRow], LOW);
  prevLedRow = ledRow;
  ledRow++;

  return true;
}

RPI_PICO_Timer ITimer0(0);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  attachMotor();
  pinMode(LDR_L_PIN, INPUT_PULLUP);
  pinMode(LDR_R_PIN, INPUT_PULLUP);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  if (ITimer0.attachInterruptInterval(3 * 1000, TimerHandler0))
  {
    Serial.print(F("Starting ITimer0 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer0. Select another freq. or timer"));
}

void loop(){
   drawScreen(sunny);
   hcsr();

  LDR_L = analogRead(LDR_L_PIN);
  //Serial.print("LDR_L : ");
  //Serial.println(LDR_L);

  LDR_R = analogRead(LDR_R_PIN);
  //Serial.print("LDR_R : ");
  //Serial.println(LDR_R);

  if((LDR_L >= LDR_THRESHOLD) && (LDR_R >= LDR_THRESHOLD)){
    if((LDR_R - LDR_L) >= LDR_TOLERANCE){
      Right(150);
    }
    else if((LDR_L - LDR_R) >= LDR_TOLERANCE){
      Left(150);
    }
    else {
      if (distance >= 15)
        Forward(255);
      else
        Stop();
    }
  }
  else{
	  Stop();
  }
}
