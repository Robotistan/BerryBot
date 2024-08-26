// Libraries
#include <IRremote.h>

// Pin Defination
#define IR_PIN 20
#define PWM_A 15
#define PWM_B 21
#define MOTOR_B1 22
#define MOTOR_B2 23
#define MOTOR_A2 24
#define MOTOR_A1 25

//IR Button Numbers
#define number_1 69
#define number_2 70
#define number_3 71
#define number_4 68
#define number_5 64
#define number_6 67
#define number_7 7
#define number_8 21
#define number_9 9
#define number_0 25
#define button_up 24
#define button_down 82
#define button_right 90
#define button_left 8
#define button_ok 28

#define DECODE_NEC

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

void Right(int speed)
{
	digitalWrite(MOTOR_A1, HIGH);
	digitalWrite(MOTOR_A2, LOW);
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
  attachMotor();
  IrReceiver.begin(IR_PIN);
}

void loop() {
  Stop();
  if (IrReceiver.decode()) {
    IrReceiver.resume();
    if(IrReceiver.decodedIRData.command != 0){
      Serial.println(IrReceiver.decodedIRData.command);
      if(IrReceiver.decodedIRData.command == button_up){  //Forward
        Forward(255);
        delay(500);
      }
      else if(IrReceiver.decodedIRData.command == button_down){  //Backward
        Backward(255);
        delay(500);
      }
      else if(IrReceiver.decodedIRData.command == button_left){  //Left
        Left(255);
        delay(130);
      }
      else if(IrReceiver.decodedIRData.command == button_right){  //Right
        Right(255);
        delay(130);
      }
      else{ //Stop
        Stop();
      }
      IrReceiver.decodedIRData.command = 0;
    }
  }
}
