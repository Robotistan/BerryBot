##########Libraries##########
from time import sleep
from machine import Pin, PWM, UART, Timer, ADC
import time, utime
from berrybot import TB6612
##########Pin Defination##########
MOTOR_A1_PIN = 25
MOTOR_A2_PIN = 24
MOTOR_B1_PIN = 22
MOTOR_B2_PIN = 23
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21
LEFT_TRACKER = 26
RIGHT_TRACKER = 27
MODE_BUTTON = 10
#############Variables####################
Mid_Speed = 58000
Low_Speed = 43000

Max_Speed = 65000
Mid_Speed = 50000
Low_Speed = 47000

STOP = 0
FWD = 1
LEFT = 2
RIGHT = 3
BWD = 4

TRACKER_THRESHOLD = 60000
directionStt = STOP
oldDirection = STOP

##########Pin Initialization##########
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
leftSensor = ADC(Pin(LEFT_TRACKER))
rightSensor = ADC(Pin(RIGHT_TRACKER))
push_button = Pin(MODE_BUTTON,Pin.IN,Pin.PULL_DOWN)

while (push_button.value()==0):
    motor.stop()
    
while True:
    leftSensorValue = leftSensor.read_u16()
    rightSensorValue = rightSensor.read_u16()
    #print(leftSensorValue)
    #print(rightSensorValue)
    #sleep(0.01)

    if leftSensorValue >= TRACKER_THRESHOLD and rightSensorValue >= TRACKER_THRESHOLD:
        directionStt = FWD
    elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue > TRACKER_THRESHOLD:
        directionStt = RIGHT
    elif leftSensorValue > TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD:
        directionStt = LEFT
    elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD and directionStt != STOP:
        directionStt = BWD
        
    if directionStt != oldDirection:
        oldDirection = directionStt
        if directionStt == FWD:
            motor.move(FWD, Mid_Speed)    
        elif directionStt == RIGHT:
            motor.move(RIGHT, Mid_Speed)
        elif directionStt == LEFT:
            motor.move(LEFT, Mid_Speed)
        elif directionStt == BWD:
            motor.move(BWD, Low_Speed)   
