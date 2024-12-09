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
#define LEFT_SENSOR 26
#define RIGHT_SENSOR 27

#define STOP  0
#define FWD   1
#define BWD   2
#define RIGHT 3
#define LEFT  4

#define TRACKER_THRESHOLD 500

// Variables
int leftSensor = 0;
int rightSensor = 0;
unsigned long reverseTime = 0;
uint8_t directionStt = STOP;
uint8_t oldDirection = STOP;

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
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(MODE_BUTTON, OUTPUT);

  while (digitalRead(MODE_BUTTON)==0){
    move(STOP,0);
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
  } 
  else if (leftSensor < TRACKER_THRESHOLD && rightSensor > TRACKER_THRESHOLD) {
    directionStt = LEFT;
  } 
  else if (leftSensor > TRACKER_THRESHOLD && rightSensor < TRACKER_THRESHOLD) {
    directionStt = RIGHT;
  } 
  else if (leftSensor < TRACKER_THRESHOLD && rightSensor < TRACKER_THRESHOLD && directionStt != STOP) {
    directionStt = BWD;
  }

  if (directionStt != oldDirection) {
    oldDirection = directionStt;
    if (directionStt == FWD)
      move(FWD, 220);
    else if (directionStt == RIGHT)
      move(RIGHT, 220);
    else if (directionStt == LEFT)
      move(LEFT, 220);
    else if (directionStt == BWD) {
      reverseTime = millis();
    }
  }
}
