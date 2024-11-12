// Pin Defination
#define TRIG_PIN 8
#define ECHO_PIN 9
#define MODE_BUTTON 10
#define BUZZER_PIN 14
#define PWM_A 15
#define PWM_B 21
#define MOTOR_B1 22
#define MOTOR_B2 23
#define MOTOR_A2 24
#define MOTOR_A1 25
#define LDR_R_PIN 28
#define LDR_L_PIN 29

#define STOP  0
#define FWD   1
#define BWD   2
#define RIGHT 3
#define LEFT  4

#define LDR_THRESHOLD 250
#define LDR_TOLERANCE 75

// Variables
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

void setup() {
  Serial.begin(115200);

  attachMotor();
  pinMode(LDR_L_PIN, INPUT_PULLUP);
  pinMode(LDR_R_PIN, INPUT_PULLUP);
  pinMode(MODE_BUTTON, OUTPUT);

  while (digitalRead(MODE_BUTTON)==0){
    move(STOP,0);
  }
}

void loop(){
  delay(10);
  LDR_L = analogRead(LDR_L_PIN);
  //Serial.print("LDR_L : ");
  //Serial.println(LDR_L);

  LDR_R = analogRead(LDR_R_PIN);
  //Serial.print("LDR_R : ");
  //Serial.println(LDR_R);

  if((LDR_R - LDR_L) >= LDR_TOLERANCE){
    move(RIGHT, 150);
  }
  else if((LDR_L - LDR_R) >= LDR_TOLERANCE){
    move(LEFT, 150);
  }
  else if((LDR_L >= LDR_THRESHOLD) && (LDR_R >= LDR_THRESHOLD)){
    move(FWD, 255);
  }
  else{
    move(STOP,0);
  }
}
