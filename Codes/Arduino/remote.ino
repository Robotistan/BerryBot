// Libraries
#include <IRremote.h>
// Pin Defination
#define TX_PIN 0
#define RX_PIN 1
#define MODE_BUTTON 10
#define BUZZER_PIN 14
#define PWM_A 15
#define PWM_B 21
#define MOTOR_B1 22
#define MOTOR_B2 23
#define MOTOR_A2 24
#define MOTOR_A1 25

#define STOP  0
#define FWD   1
#define BWD   2
#define RIGHT 3
#define LEFT  4

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

void move (int direction, int speed){
  if (direction == FWD){
    analogWrite(PWM_A, speed);
    analogWrite(PWM_B, speed);

    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
  }
  else if (direction == BWD){
    analogWrite(PWM_A, speed);
    analogWrite(PWM_B, speed);

    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
  }
  else if (direction == RIGHT){
    analogWrite(PWM_A, speed);
    analogWrite(PWM_B, speed);

    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
  }
  else if (direction == LEFT){
    analogWrite(PWM_A, speed);
    analogWrite(PWM_B, speed);
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
  }
  else if (direction == STOP){
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);

    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, LOW);
  }
}

void berry_horn() {
  for(int i=0; i<50; i++){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(2);
    digitalWrite(BUZZER_PIN, LOW);
    delay(2);
  }
}

void setup() {
  Serial.begin(115200);

  attachMotor();
  pinMode(MODE_BUTTON, OUTPUT);

  while (digitalRead(MODE_BUTTON)==0){
    move(STOP,0);
  }
}

void loop(){
  if (IrReceiver.decode()) {
    IrReceiver.resume();
    if(IrReceiver.decodedIRData.command != 0){
      //Serial.println(IrReceiver.decodedIRData.command);
      if(IrReceiver.decodedIRData.command == button_up){  //Forward
        move(FWD,255);
        delay(500);
        move(STOP,0);
      }
      else if(IrReceiver.decodedIRData.command == button_down){  //Backward
        move(BWD,255);
        delay(500);
        move(STOP,0);
      }
      else if(IrReceiver.decodedIRData.command == button_left){  //Left
        move(LEFT,255);
        delay(130);
        move(STOP,0);
      }
      else if(IrReceiver.decodedIRData.command == button_right){  //Right
        move(RIGHT,255);
        delay(130);
        move(STOP,0);
      }
      else if(IrReceiver.decodedIRData.command == number_1){  //Buzzer
        berry_horn();
      }
      else{ //Stop
        move(STOP,0);
      }
      IrReceiver.decodedIRData.command = 0;
    }
  }
}
