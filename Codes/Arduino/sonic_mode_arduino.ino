#include <Wire.h>
#include <stdio.h>


// Pin Defination
#define TX_PIN 0
#define RX_PIN 1
#define NEOPIXEL_PIN 6
#define TRIG_PIN 8
#define ECHO_PIN 9
#define MODE_BUTTON 10
#define BUZZER_PIN 14
#define PWM_A 15
#define IR_PIN 20
#define PWM_B 21
#define INPUT_B1 22
#define INPUT_B2 23
#define INPUT_A2 24
#define INPUT_A1 25
#define LEFT_SENSOR 26
#define RIGHT_SENSOR 27
#define LDR_R_PIN 28
#define LDR_L_PIN 29

// Variable
long duration;
int distance;



void attachMotor()
{
  pinMode(INPUT_A1, OUTPUT);
  pinMode(INPUT_A2, OUTPUT);
  pinMode(INPUT_B1, OUTPUT);
  pinMode(INPUT_B2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
}

void Forward(int speed)
{
	digitalWrite(INPUT_A1, HIGH);
	digitalWrite(INPUT_A2, LOW);
  digitalWrite(INPUT_B1, HIGH);
	digitalWrite(INPUT_B2, LOW);
  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

void Backward(int speed)
{
	digitalWrite(INPUT_A1, LOW);
	digitalWrite(INPUT_A2, HIGH);
  digitalWrite(INPUT_B1, LOW);
	digitalWrite(INPUT_B2, HIGH);
  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

void Left(int speed)
{
	digitalWrite(INPUT_A1, HIGH);
	digitalWrite(INPUT_A2, LOW);
	digitalWrite(INPUT_B1, LOW);
  digitalWrite(INPUT_B2, HIGH);
  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

void Right(int speed)
{
  digitalWrite(INPUT_A1, LOW);
	digitalWrite(INPUT_A2, HIGH);
  digitalWrite(INPUT_B1, HIGH);
	digitalWrite(INPUT_B2, LOW);
  analogWrite(PWM_A, speed);
  analogWrite(PWM_B, speed);
}

void Stop()
{
	digitalWrite(INPUT_A1, LOW);
	digitalWrite(INPUT_A2, LOW);
  digitalWrite(INPUT_B1, LOW);
	digitalWrite(INPUT_B2, LOW);
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
  delay(1999);

  pinMode(BUZZER_PIN, OUTPUT);

  attachMotor();
  pinMode(LDR_L_PIN, INPUT_PULLUP);
  pinMode(LDR_R_PIN, INPUT_PULLUP);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(MODE_BUTTON, OUTPUT);

  
}
void loop(){
  hcsr();
  int cm=distance;
  if(cm>30){
    Forward(200);
  }
  else{
    Stop();
    delay(1000);
    Backward(200);
    delay(500);
    Stop();
    delay(200);
    Left(120);
    delay(300);
    Stop();
    delay(200);
  }
}