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

#define STOP  0
#define FWD   1
#define BWD   2
#define RIGHT 3
#define LEFT  4

// Variables
long duration;
int left_counter=0;

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

long hcsr() {
  long distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH);

  distance = (duration / 2) / 29.1;

  return distance;
}

void setup() {
  Serial.begin(115200);

  attachMotor();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MODE_BUTTON, OUTPUT);

  while (digitalRead(MODE_BUTTON)==0){
    move(STOP,0);
  }
}

void loop(){
  long distance = hcsr();

  if(distance > 12){
    move(FWD, 255);
  }
  else{
    move(STOP,0);
    delay(500);
    move(BWD, 200);
    delay(100);
    move(STOP, 0);
    delay(200);
    move(LEFT, 200);
    if(left_counter==0){
      delay(500);
      left_counter++;
    }
    else{
      delay(800);
      left_counter=0;
    }
    move(STOP,0);
    delay(200);
  }
}
