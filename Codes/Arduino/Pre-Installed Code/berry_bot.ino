// You must select the "Generic RP2040" board from the Arduino IDE board manager

// Libraries
#include <Wire.h>
#include <picobricks.h>
#include <stdio.h>
#include <IRremote.h>
#include "RPi_Pico_TimerInterrupt.h"

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
#define MOTOR_B1 22
#define MOTOR_B2 23
#define MOTOR_A2 24
#define MOTOR_A1 25
#define LEFT_SENSOR 26
#define RIGHT_SENSOR 27
#define LDR_R_PIN 28
#define LDR_L_PIN 29

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
// Define constants
#define DECODE_NEC
#define PIXEL_INTERVAL 100
#define LDR_THRESHOLD 250
#define LDR_TOLERANCE 75
#define TRACKER_THRESHOLD 700
#define RGB_COUNT 6

#define STOP  0
#define FWD   1
#define BWD   2
#define RIGHT 3
#define LEFT  4

#define SERVICE_UUID           "AT+BLESERUUID=6E400001B5A3F393E0A9E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "AT+BLERXUUID=6E400002B5A3F393E0A9E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "AT+BLETXUUID=6E400003B5A3F393E0A9E50E24DCCA9E"

#define BLE_BLINK_INTERVAL  500

int rowPins[5] = {7, 11, 12, 13, 17}; //Row LedMatrix Pins
int colPins[5] = {18, 19, 16, 2, 3};  //Col LedMatrix Pins

// Variables
long duration;
int distance;
int ble_data;
int ble_cnt = 0;
int ble_buf[10];
float LDR_L;
float LDR_R;
int pixelNumber = 0;
int pixelMode = 0;
int BerryMode = 0;
int leftSensor = 0;
int rightSensor = 0;
int bleConnected = 0;
int ledTime = 0;
int ledWasOn = 0;
int rgb_status = 1;
int led_matrix_status = 0;
int i =0;
int rgb_value[6][3]= { {0, 0, 127}, 
	                     {0, 0, 127}, 
                       {0, 0, 127}, 
                       {0, 0, 127},
                       {0, 0, 127},
                       {0, 0, 127} };

unsigned long currentTime = millis();
unsigned long pixelChangeTime = 0;
unsigned long reverseTime = 0;
uint8_t directionStt = STOP;
uint8_t oldDirection = STOP;
int buttonState = 0;
int lastButtonState = 0;
int user_speed = 0;
int avg = 0;
int left_counter=0;
int counter = 0;
int bluetooth_mode = 0;
int direction = 0;
int right_speed = 0;
int left_speed = 0;

// Define the LED matrix symbols
int smile[5] =     {0x0A,0x0A,0x00,0x11,0x0E};
int yes[5] =       {0x00,0x01,0x02,0x14,0x08};
int no[5] =        {0x11,0x0A,0x04,0x0A,0x11};
int left[5] =      {0x04,0x0E,0x15,0x04,0x04};
int right[5] =     {0x04,0x04,0x15,0x0E,0x04};
int forward[5] =   {0x04,0x02,0x1F,0x02,0x04};
int backward[5] =  {0x04,0x08,0x1F,0x08,0x04};
int empty[5] =     {0x00,0x00,0x00,0x00,0x00};
int full[5] =      {0x1F,0x1F,0x1F,0x1F,0x1F};
int heart[5] =     {0x0A,0x1F,0x1F,0x0E,0x04};
int sad[5] =       {0x00,0x0A,0x00,0x0E,0x11};
int tracker[5] =   {0x07,0x04,0x1F,0x04,0x07};
int bluetooth[5] = {0x1F,0x00,0x0E,0x00,0x04};
int ir[5] =        {0x1F,0x11,0x1F,0x11,0x1F};
int sunny[5] =     {0x15,0x0E,0x1F,0x0E,0x15};
int sonic[5] =     {0x04,0x0E,0x1B,0x0E,0x04};
int triangle[5] =  {0x1F,0x11,0x11,0x0A,0x04};
int user_led_matrix[5];

// Function Declaration
NeoPixel strip (NEOPIXEL_PIN, RGB_COUNT);

// Function to initialize motor pins
void attachMotor()
{
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
}

// Function to set motor speeds (for left and right motors)
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (rightSpeed > 0) {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    if (rightSpeed < 100)
      rightSpeed = 100;
    analogWrite(PWM_A, rightSpeed);
  } 
  else {
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    if (-rightSpeed < 100)
      rightSpeed = -100;
    analogWrite(PWM_A, -rightSpeed);
  }

  if (leftSpeed > 0) {
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
    if (leftSpeed < 100)
      leftSpeed = 100;
    analogWrite(PWM_B, leftSpeed);
  } 
  else {
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
    if (-leftSpeed < 100)
      leftSpeed = -100;
    analogWrite(PWM_B, -leftSpeed);
  }
}

// Function to move the robot in different directions
void move (int direction, int speed){
  if (direction == FWD){  // Forward
    analogWrite(PWM_A, speed);
    analogWrite(PWM_B, speed);

    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
  }
  else if (direction == BWD){  // Backward
    analogWrite(PWM_A, speed);
    analogWrite(PWM_B, speed);

    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
  }
  else if (direction == RIGHT){  // Right
    analogWrite(PWM_A, speed);
    analogWrite(PWM_B, speed);

    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
  }
  else if (direction == LEFT){  // Left
    analogWrite(PWM_A, speed);
    analogWrite(PWM_B, speed);
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
  }
  else if (direction == STOP){  // Stop
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);

    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, LOW);
  }
}

// Function to sound the horn
void berry_horn() {
  for(int i=0; i<50; i++){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(2);
    digitalWrite(BUZZER_PIN, LOW);
    delay(2);
  }
}

// Function for BLE response handling
void BLEReadResponse() {
  int timeout = 150;
  while (!Serial1.available() && timeout > 0) {
    timeout--;
    delay(20);
  }
  while (Serial1.available()) {
    String r = Serial1.readStringUntil(0x0a);
    Serial.print(r);
    delay(15);
  }
}

// Function to configure BLE
void BLEConfigure() {
  Serial1.begin(115200);
  Serial1.write("AT+BLENAME=BerryBot");
  Serial1.write("\r\n");
  BLEReadResponse();
  delay(500);
  Serial.println("\n-------");
  Serial1.write(SERVICE_UUID);
  Serial1.write("\r\n");
  BLEReadResponse();
  delay(500);
  Serial.println("\n-------");
  Serial1.write(CHARACTERISTIC_UUID_RX);
  Serial1.write("\r\n");
  BLEReadResponse();
  delay(500);
  Serial.println("\n-------");
  Serial1.write(CHARACTERISTIC_UUID_TX);
  Serial1.write("\r\n");
  BLEReadResponse();
  delay(500);
  Serial.println("\n-------");
  Serial1.write("AT+SYSIOMAP=1,4");
  Serial1.write("\r\n");
  BLEReadResponse();
  delay(500);
  Serial.println("\n-------");
}

volatile int ledRow = 0;
volatile int prevLedRow = 4;
volatile byte ledArrayBuffer[5];

// Function to draw LED matrix screen based on a buffer array
void drawScreen(int buffer[]){
  for (i = 0; i<5; i++) {
      ledArrayBuffer[i] = buffer[i];
  }
}

// Function to set LED matrix columns
void setColumns(byte b) {
    digitalWrite(colPins[0], (~b >> 0) & 0x01); 
    digitalWrite(colPins[1], (~b >> 1) & 0x01); 
    digitalWrite(colPins[2], (~b >> 2) & 0x01); 
    digitalWrite(colPins[3], (~b >> 3) & 0x01); 
    digitalWrite(colPins[4], (~b >> 4) & 0x01); 
}

// Timer interrupt handler function for controlling LED matrix
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

// Function to control the NeoPixel LED strip in various modes
void pixelFunction() {
  if (pixelMode == 0) {
    strip.setPixelColor(pixelNumber, rgb_value[pixelNumber][0], rgb_value[pixelNumber][1], rgb_value[pixelNumber][2]);
    pixelNumber++;
    if (pixelNumber > 6) {
      pixelNumber = 0;
      pixelMode = 1;
    }
  }
  if (pixelMode == 1) {
    pixelNumber++;
    if (pixelNumber > 6) {
      pixelNumber = 0;
      pixelMode = 2;
    }
  }
  if (pixelMode == 2) {
    strip.setPixelColor(pixelNumber++, 0, 0, 0);
    if (pixelNumber > 6) {
      pixelNumber = 0;
      pixelMode = 3;
    }
  }
  if (pixelMode == 3) {
    pixelNumber++;
    if (pixelNumber > 6) {
      pixelNumber = 0;
      pixelMode = 0;
    }
  }
}

// Function to handle the NeoPixel LED strip updates based on a defined interval
void rgbFunction(){
  currentTime = millis();
  if (currentTime - pixelChangeTime > PIXEL_INTERVAL) {
    pixelFunction();
    pixelChangeTime = currentTime;
  }
}

// Function to handle ultrasonic sensor (HC-SR04) measurement
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

// Function to handle BLE connection status and control the onboard LED for connection indication
void bleConnect (){
  if (bleConnected == false) {
    if (millis() - ledTime > BLE_BLINK_INTERVAL) {
      ledTime = millis();  // Update the blink time
      if (ledWasOn == 0) {    // LED ON AT command
        Serial1.write("AT+SYSGPIOWRITE=0,0");
        Serial1.write("\r\n");
        ledWasOn = 1;  // Set LED status as on
      } 
      else {  // LED OFF AT Command
        Serial1.write("AT+SYSGPIOWRITE=0,1");
        Serial1.write("\r\n");
        ledWasOn = 0;  // Set LED status as off
      }
    }
  }
  else{  // Keep the LED on if BLE is connected
    Serial1.write("AT+SYSGPIOWRITE=0,0");
    Serial1.write("\r\n");
    ledWasOn = 1;
  }
}

// Interrupt handler for the MODE_BUTTON. Used to switch modes when the button is pressed.
void buttonInterruptHandler() {
  buttonState = digitalRead(MODE_BUTTON);
  if ((buttonState == HIGH) && (lastButtonState == 0)) {
    lastButtonState = 1;
    move(STOP,0);
    if (BerryMode >= 6){
      BerryMode = 0;
    }
    else{
      BerryMode++;
    }
    delay(500);
  }
  else if ((buttonState == LOW) && (lastButtonState == 1)){
	lastButtonState = 0;
	delay(500);
  }
}

RPI_PICO_Timer ITimer0(0);  // Timer interrupt for controlling LED matrix

// Function to track light levels from two LDR sensors and adjust movement accordingly
void light_tracker(){
  delay(10);
  LDR_L = analogRead(LDR_L_PIN);
  //Serial.print("LDR_L : ");
  //Serial.println(LDR_L);

  LDR_R = analogRead(LDR_R_PIN);
  //Serial.print("LDR_R : ");
  //Serial.println(LDR_R);

  // If there is significant difference in LDR values, turn the robot accordingly
  if((LDR_R - LDR_L) >= LDR_TOLERANCE){
    move(RIGHT, 150);  // Move right if right LDR value is higher
  }
  else if((LDR_L - LDR_R) >= LDR_TOLERANCE){
    move(LEFT, 150);  // Move left if left LDR value is higher
  }
  else if((LDR_L >= LDR_THRESHOLD) && (LDR_R >= LDR_THRESHOLD)){
    move(FWD, 255);   // Move forward if both LDR values are high
  }
  else{
    move(STOP,0);  // Stop if both LDR values are low
  }
}

// Function to follow a line using two sensors (left and right)
void line_tracker(){
  leftSensor = analogRead(LEFT_SENSOR);
  rightSensor = analogRead(RIGHT_SENSOR);

  // Determine direction based on sensor readings
  if (leftSensor >= TRACKER_THRESHOLD && rightSensor >= TRACKER_THRESHOLD) {
    directionStt = FWD;  // Move forward if both sensors detect the line
  } 
  else if (leftSensor < TRACKER_THRESHOLD && rightSensor > TRACKER_THRESHOLD) {
    directionStt = LEFT;  // Turn left if only right sensor detects the line
    //directionStt = RIGHT;
  } 
  else if (leftSensor > TRACKER_THRESHOLD && rightSensor < TRACKER_THRESHOLD) {
    directionStt = RIGHT;  // Turn right if only left sensor detects the line
    //directionStt = LEFT;
  } 
  else if (leftSensor < TRACKER_THRESHOLD && rightSensor < TRACKER_THRESHOLD && directionStt != STOP) {
    directionStt = BWD;  // Move backward if neither sensor detects the line
  }

  // Move in the detected direction
  if (directionStt != oldDirection) {
    oldDirection = directionStt;
    if (directionStt == FWD)    // Forward
      move(FWD, 220);
    else if (directionStt == RIGHT)
      move(RIGHT, 220);
    else if (directionStt == LEFT)
      move(LEFT, 220);
    else if (directionStt == BWD) {   //Backward
      reverseTime = millis();
    }
  }
}

// Function to operate the robot in sonic mode based on distance measurements
void sonic_mode(){
  long distance = hcsr();  // Get the distance from the ultrasonic sensor

  // If distance is more than 12 cm, move forward
  if(distance > 12){
    move(FWD, 255);
  }
  else{  // If too close, stop and perform reverse and turn movements
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

// Function to handle the sumo mode, where the robot moves to avoid obstacles
void sumo(){
  long distance = hcsr();  // Get distance from ultrasonic sensor
  leftSensor = analogRead(LEFT_SENSOR);     // Read left sensor
  rightSensor = analogRead(RIGHT_SENSOR);  // Read right sensor

  if (distance <= 22) {  // If object is too close
    if ((leftSensor >= TRACKER_THRESHOLD) || (rightSensor >= TRACKER_THRESHOLD)){
      move(BWD,255);
      delay(500);
    }
    else if ((leftSensor < TRACKER_THRESHOLD) && (rightSensor < TRACKER_THRESHOLD)){
      move(FWD,255);
      delay(100);
    }
    else
      move(STOP,0);
  }
  else{
    if ((leftSensor >= TRACKER_THRESHOLD) || (rightSensor >= TRACKER_THRESHOLD)){
      move(BWD,255);
      delay(200);
    }
    else if ((leftSensor < TRACKER_THRESHOLD) && (rightSensor < TRACKER_THRESHOLD)){
      counter += 1;
      if (counter == 3){
        move(FWD,180);
        delay(100);
        counter = 0;
      }
      else{
        move(LEFT,180);
        delay(100);
        move(STOP,0);
      }
    }else
      move(STOP,0);
  }
}

void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud rate
  Wire.begin();  // Start the I2C communication
  delay(1999);
  BLEConfigure();  // Configure the Bluetooth communication
  pinMode(BUZZER_PIN, OUTPUT);
  attachMotor();  // Initialize motor control pins
  pinMode(LDR_L_PIN, INPUT_PULLUP);
  pinMode(LDR_R_PIN, INPUT_PULLUP);
  IrReceiver.begin(IR_PIN);  // Initialize IR receiver
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // Set up the columns and rows for the LED matrix
  for (int i = 0; i < 5; i++)
  {
    pinMode(colPins[i], OUTPUT);       
    pinMode(rowPins[i], OUTPUT);
  }
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(MODE_BUTTON, OUTPUT);
  // Attach an interrupt to the MODE_BUTTON to handle mode switching
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), buttonInterruptHandler, CHANGE);
  // Set up a timer interrupt to handle tasks at regular intervals
  if (ITimer0.attachInterruptInterval(3 * 1000, TimerHandler0))
  {
    Serial.print(F("Starting ITimer0 OK, millis() = ")); Serial.println(millis());
  }
  else
    Serial.println(F("Can't set ITimer0. Select another freq. or timer"));

  drawScreen(smile);
}

void loop() {
  switch(BerryMode){
    case 0: //Bluetooth Settings
      move(STOP,0);
      drawScreen(smile);
      rgbFunction();
      bleConnect();  
      Serial1.write("+++");
      delay(300);
      Serial1.write("AT+TRANSENTER");
      Serial1.write("\r\n");
      BLEReadResponse();
      ble_data = 0;
      BerryMode++;
      break;
    case 1: //Bluetooth Mode
      if(led_matrix_status == 0){
        drawScreen(bluetooth);
      }
      else if (led_matrix_status == 1){
        drawScreen(user_led_matrix);
      }

      if(rgb_status == 1){
        rgbFunction();
      }

      ble_data = Serial1.read();  // Read the Bluetooth data
      
      if(ble_data == -1){
        ble_data = 0;
        ble_cnt = 0;
        for(int i=0; i<10; i++){
          ble_buf[i] = 0xFF;  // Reset the buffer if no data
        }
      }
      else if (ble_data == 82){  // Handle Bluetooth Header command
        ble_cnt = 0;
        ble_buf[0] = ble_data;
        for(int i=1; i<10; i++){
          ble_buf[i] = 0xFF;
        }
        ble_cnt++;
        ble_data = 0;
      }
      else{  // Store the Bluetooth data in the buffer
        ble_buf[ble_cnt] = ble_data;
        ble_cnt++;
        ble_data = 0;
      }

      if (ble_data == '+') {  // Check if the BLE device has disconnected
        String bleStr = Serial1.readStringUntil(0x0A);
        if (bleStr.indexOf("BLE_DISCONNECT") > -1) {
          ble_data = 0;
          bleConnected = false;
        }
        else{
          bleConnected = true;
        }
      }

      bleConnect();  // Handle different Bluetooth commands

      if ((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 99)){  //Exit modes
        bluetooth_mode = 0;
        move(STOP,0);
      }
      else if((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 1) && (ble_buf[3] == 0)){ //Neo turn off
        bluetooth_mode = 0;
        rgb_status = 0;
        for(i=0; i<6; i++){
          strip.setPixelColor(i, 0, 0, 0);
          delay(50);
        }
      }
      else if((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 90) && (ble_buf[3] == 0)){ //Neo turn on
        bluetooth_mode = 0;
        rgb_status = 1;
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 2) && (ble_buf[3] == 0)){ //Horn
        bluetooth_mode = 0;
        berry_horn();
      }
      else if (((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 4) && (ble_buf[3] == 0)) || (bluetooth_mode == 1)){ //Sonic
        bluetooth_mode = 1;
        sonic_mode();
      }
      else if (((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 8) && (ble_buf[3] == 0)) || (bluetooth_mode == 2)){ //Line Tracker
        bluetooth_mode = 2;
        line_tracker();
      }
      else if (((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 16) && (ble_buf[3] == 0)) || (bluetooth_mode == 3)){ //Light Tracker
        bluetooth_mode = 3;
        light_tracker();
      }
      else if (((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 32) && (ble_buf[3] == 0)) || (bluetooth_mode == 4)){ //Sumo
        bluetooth_mode = 4;
        sumo();
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 3) && (ble_buf[2] != 255) && (ble_buf[3] != 255)){ //Move
        bluetooth_mode = 0;
        led_matrix_status = 2;

        if((ble_buf[2] == 0) && (ble_buf[3] == 0)){
          led_matrix_status = 0;
          move(STOP,0);
          drawScreen(bluetooth);
        }
        else{
          int joyX = ble_buf[2];
          int joyY = ble_buf[3];
          int mappedX = map(joyX, 0, 255, -255, 255);
          int mappedY = map(joyY, 0, 255, -255, 255);
          int leftMotorSpeed = mappedY + mappedX;
          int rightMotorSpeed = mappedY - mappedX;

          leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
          rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
          setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
        }
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 7)){ //RGB Menu
        bluetooth_mode = 0;

        rgb_value[ble_buf[2]-1][0] = ble_buf[3];
        rgb_value[ble_buf[2]-1][1] = ble_buf[4];
        rgb_value[ble_buf[2]-1][2] = ble_buf[5];
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 6) && (ble_buf[2] != 255) && (ble_buf[3] != 255) && (ble_buf[4] != 255) && (ble_buf[5] != 255) && (ble_buf[6] != 255)){  //Led Matrix 
        bluetooth_mode = 0;

        user_led_matrix[0] = ble_buf[2];
        user_led_matrix[1] = ble_buf[3];
        user_led_matrix[2] = ble_buf[4];
        user_led_matrix[3] = ble_buf[5];
        user_led_matrix[4] = ble_buf[6];

        led_matrix_status = 1;
        drawScreen(user_led_matrix);
      }
      else{
        delay(10);
      }
      break;
    case 2: //IR Mode
      drawScreen(ir);
      rgbFunction();
      bleConnect();
      if (IrReceiver.decode()) {
        IrReceiver.resume();
        if(IrReceiver.decodedIRData.command != 0){
          //Serial.println(IrReceiver.decodedIRData.command);
          if(IrReceiver.decodedIRData.command == button_up){  //Forward
            drawScreen(forward);
            move(FWD,255);
            delay(500);
            move(STOP,0);
          }
          else if(IrReceiver.decodedIRData.command == button_down){  //Backward
            drawScreen(backward);
            move(BWD,255);
            delay(500);
            move(STOP,0);
          }
          else if(IrReceiver.decodedIRData.command == button_left){  //Left
            drawScreen(left);
            move(LEFT,255);
            delay(130);
            move(STOP,0);
          }
          else if(IrReceiver.decodedIRData.command == button_right){  //Right
            drawScreen(right);
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
      break;
    case 3:  //Sumo Mode
      drawScreen(triangle);
      rgbFunction();
      bleConnect();
      sumo();
      break;
    case 4: //Line Tracker Mode
      drawScreen(tracker);
      rgbFunction();
      bleConnect();
      line_tracker();
      break;
    case 5: //Light Tracker Mode
      drawScreen(sunny);
      rgbFunction();
      bleConnect();
      light_tracker();
      break;
    case 6:  //Sonic Mode
      drawScreen(sonic);
      delay(100);
      rgbFunction();
      bleConnect();
      sonic_mode();
      break;
  }
}
