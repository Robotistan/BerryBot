// Libraries
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
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
#define INPUT_B1 22
#define INPUT_B2 23
#define INPUT_A2 24
#define INPUT_A1 25
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

#define DECODE_NEC
#define PIXEL_INTERVAL 100
#define LDR_THRESHOLD 250
#define LDR_TOLERANCE 75
#define TRACKER_THRESHOLD 1200

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

// Variable
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

// Led Matrix
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
int user_led_matrix[5];

// Function Declaration
Adafruit_NeoPixel strip(6, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

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

void drawScreen(int buffer[]){
  for (i = 0; i < 5; i++) {
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

void fullPixel(){
  for(int i=0;i<6;i++){
    strip.setPixelColor(i, 0, 0, 127);
  }
  strip.show();
}

void pixelFunction() {
  if (pixelMode == 0) {
    strip.setPixelColor(pixelNumber, rgb_value[pixelNumber][0], rgb_value[pixelNumber][1], rgb_value[pixelNumber][2]);
    pixelNumber++;
    strip.show();
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
    strip.show();
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

void rgbFunction(){
  currentTime = millis();
  if (currentTime - pixelChangeTime > PIXEL_INTERVAL) {
    pixelFunction();
    pixelChangeTime = currentTime;
  }
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

void bleConnect (){
  if (bleConnected == false) {
    if (millis() - ledTime > BLE_BLINK_INTERVAL) {
      ledTime = millis();
      if (ledWasOn == 0) {    // LED ON AT command
        Serial1.write("AT+SYSGPIOWRITE=0,0");
        Serial1.write("\r\n");
        ledWasOn = 1;
      } 
      else {  // LED OFF AT Command
        Serial1.write("AT+SYSGPIOWRITE=0,1");
        Serial1.write("\r\n");
        ledWasOn = 0;
      }
    }
  }
}

void buttonInterruptHandler() {
  buttonState = digitalRead(MODE_BUTTON);
  if ((buttonState == HIGH) && (lastButtonState == 0)) {
    lastButtonState = 1;
    if (BerryMode >= 6){
      BerryMode = 1;
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

RPI_PICO_Timer ITimer0(0);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1999);
  BLEConfigure();
  pinMode(BUZZER_PIN, OUTPUT);
  strip.begin();
  attachMotor();
  pinMode(LDR_L_PIN, INPUT_PULLUP);
  pinMode(LDR_R_PIN, INPUT_PULLUP);
  IrReceiver.begin(IR_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  for (int i = 0; i < 5; i++)
  {
    pinMode(colPins[i], OUTPUT);       
    pinMode(rowPins[i], OUTPUT);
  }
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(MODE_BUTTON, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON), buttonInterruptHandler, CHANGE);
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
    case 0: //Choose Mode
      rgbFunction();
      bleConnect();
      break;
    case 1: //Bluetooth Settings
      drawScreen(bluetooth);
      rgbFunction();
      bleConnect();
      Serial1.write("+++");
      delay(300);
      Serial1.write("AT+TRANSENTER");
      Serial1.write("\r\n");
      BLEReadResponse();
      ble_data = 0;
      BerryMode = 2;
      break;
    case 2: //Bluetooth Mode
      if(led_matrix_status == 0){
        drawScreen(bluetooth);
      }
      else if (led_matrix_status == 1){
        drawScreen(user_led_matrix);
      }

      if(rgb_status == 1){
        rgbFunction();
      }

      ble_data = Serial1.read();
      //Serial.println(ble_data);
      //delay(1500);
      delay(100);
      if(ble_data == -1){
        ble_data = 0;
        ble_cnt = 0;
        for(int i=0; i<10; i++){
          ble_buf[i] = 0xFF;
        }
      }
      else if (ble_data == 82){
        ble_cnt = 0;
        ble_buf[0] = ble_data;
        for(int i=1; i<10; i++){
          ble_buf[i] = 0xFF;
        }
        ble_cnt++;
        ble_data = 0;
      }
      else{
        ble_buf[ble_cnt] = ble_data;
        ble_cnt++;
        ble_data = 0;
      }
      
      //while(!(Serial1.available()));
      if (ble_data == '+') {
        String bleStr = Serial1.readStringUntil(0x0A);
        if (bleStr.indexOf("BLE_DISCONNECT") > -1) {
          ble_data = 0;
            bleConnected = false;
          }
        else{
          bleConnected = true;
        }
      }

      bleConnect();
      delay(50);
/*
        Serial.println("------");
        Serial.println(ble_buf[2]);
        Serial.println(ble_buf[3]);
        Serial.println("------");
*/
      if((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 1) && (ble_buf[3] == 0)){ //Neo turn on/off
        if(rgb_status == 1){
          rgb_status = 0;
          for(i=0; i<6; i++){
            strip.setPixelColor(i, 0, 0, 0);
            strip.show();
            delay(50);
          }
          strip.show();
        }
        else{
          rgb_status = 1;
        }
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 2) && (ble_buf[3] == 0)){ //Horn
        //Serial.println("horn");
        tone(BUZZER_PIN, 500);
        delay(1000);
        noTone(BUZZER_PIN);
        delay(1000);
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 4) && (ble_buf[3] == 0)){ //Sonic
        //Serial.println("Sonic");
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 8) && (ble_buf[3] == 0)){ //Tracker
        //Serial.println("Tracker");
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 2) && (ble_buf[2] == 16) && (ble_buf[3] == 0)){ //Light Sensor
        //Serial.println("Light Sensor");
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 3) && (ble_buf[2] != 255) && (ble_buf[3] != 255)){ //Move
        led_matrix_status = 2;
        
        if((ble_buf[3] >= 1) && (ble_buf[3] <= 90)){ //Forward
          drawScreen(forward);
          user_speed = constrain(ble_buf[3],100,254);
          Forward(user_speed);
          //Forward(255);
          delay(500);
        }
        else if((ble_buf[3] >= 160) && (ble_buf[3] <= 254)){ //Backward
          drawScreen(backward);
          user_speed = constrain(ble_buf[3],100,254);
          Backward(user_speed);
          //Backward(255);
          delay(500);
        }
        else if((ble_buf[2] >= 1) && (ble_buf[2] <= 90)){ //Left
          drawScreen(left);
          user_speed = constrain(ble_buf[2],100,254);
          Left(user_speed);
          //Left(255);
          delay(130);
        }
        else if((ble_buf[2] >= 160) && (ble_buf[2] <= 254)){ //Right
          drawScreen(right);
          user_speed = constrain(ble_buf[2],100,254);
          Right(user_speed);
          //Right(255);
          delay(130);
        }
        else{ //Stop
          led_matrix_status = 0;
          Stop();
          drawScreen(bluetooth);
        }
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 7)){ //RGB Menu
        rgb_value[ble_buf[2]-1][0] = ble_buf[3];
        rgb_value[ble_buf[2]-1][1] = ble_buf[4];
        rgb_value[ble_buf[2]-1][2] = ble_buf[5];
      }
      else if ((ble_buf[0] == 82) && (ble_buf[1] == 6) && (ble_buf[2] != 255) && (ble_buf[3] != 255) && (ble_buf[4] != 255) && (ble_buf[5] != 255) && (ble_buf[6] != 255)){  //Led Matrix        
        user_led_matrix[0] = ble_buf[2];
        user_led_matrix[1] = ble_buf[3];
        user_led_matrix[2] = ble_buf[4];
        user_led_matrix[3] = ble_buf[5];
        user_led_matrix[4] = ble_buf[6];

        led_matrix_status = 1;
        drawScreen(user_led_matrix);
        delay(300);
      }
      else{
        delay(10);
      }
      break;
    case 3: //IR Mode
      drawScreen(ir);
      rgbFunction();
      bleConnect();
      //hcsr();
      if (IrReceiver.decode()) {
        IrReceiver.resume();
        if(IrReceiver.decodedIRData.command != 0){
          //Serial.println(IrReceiver.decodedIRData.command);
          if(IrReceiver.decodedIRData.command == button_up){  //Forward
            drawScreen(forward);
            Forward(255);
            delay(500);
          }
          else if(IrReceiver.decodedIRData.command == button_down){  //Backward
            drawScreen(backward);
            Backward(255);
            delay(500);
          }
          else if(IrReceiver.decodedIRData.command == button_left){  //Left
            drawScreen(left);
            Left(255);
            delay(130);
          }
          else if(IrReceiver.decodedIRData.command == button_right){  //Right
            drawScreen(right);
            Right(255);
            delay(130);
          }
          else if(IrReceiver.decodedIRData.command == number_1){  //Buzzer
            tone(BUZZER_PIN, 500);
            delay(1000);
            noTone(BUZZER_PIN);
            delay(1000);
          }
          else{ //Stop
            Stop();
          }
          IrReceiver.decodedIRData.command = 0;
        }
      }
      break;
    case 4: //Light Tracker Mode
      drawScreen(sunny);
      rgbFunction();
      bleConnect();
      //hcsr();

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
      break;
    case 5: //Line Tracker Mode
      drawScreen(tracker);
      rgbFunction();
      bleConnect();

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
        directionStt = RIGHT;
      } 
      else if (leftSensor > TRACKER_THRESHOLD && rightSensor < TRACKER_THRESHOLD) {
        directionStt = LEFT;
      } 
      else if (leftSensor < TRACKER_THRESHOLD && rightSensor < TRACKER_THRESHOLD && directionStt != STOP) {
        directionStt = BWD;
      }

      if (directionStt != oldDirection) {
        oldDirection = directionStt;
        if (directionStt == FWD)
          Forward(140);
        else if (directionStt == RIGHT)
          Right(140);
        else if (directionStt == LEFT)
          Left(140);
        else if (directionStt == BWD) {
          Backward(140);
          reverseTime = millis();
        } else if (directionStt == STOP)
          Stop();
      }

      if (directionStt == BWD && millis() - reverseTime > 300)
        directionStt = STOP;
      break;
    case 6: //Sonic Mode
      drawScreen(sonic);
      rgbFunction();
      bleConnect();
      break;
  }
}
