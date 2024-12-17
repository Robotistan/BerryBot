##########Libraries##########
from machine import Pin, PWM, UART, Timer, ADC
from time import sleep
from berrybot import TB6612, HCSR04
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
Max_Speed = 65000
Mid_Speed = 50000
Low_Speed = 47000

TRACKER_THRESHOLD = 50000
counter = 0
##########Pin Initialization##########
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
sensor = HCSR04(trigger_pin=8, echo_pin=9, echo_timeout_us=10000)
leftSensor = ADC(Pin(LEFT_TRACKER))
rightSensor = ADC(Pin(RIGHT_TRACKER))
push_button = Pin(MODE_BUTTON,Pin.IN,Pin.PULL_DOWN)

while (push_button.value()==0):
    motor.stop()
    
while True:
    distance = sensor.distance_cm()
    #print(distance)

    leftSensorValue = leftSensor.read_u16()
    rightSensorValue = rightSensor.read_u16()
    #print(leftSensorValue)
    #print(rightSensorValue)
    sleep(0.02)
    if distance <= 15:
        if leftSensorValue >= TRACKER_THRESHOLD or rightSensorValue >= TRACKER_THRESHOLD:
            motor.move(BWD, Mid_Speed)
            sleep(0.5)
        elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD:
            motor.move(FWD, Mid_Speed)
            sleep(0.5)
        else:
            motor.move(STOP, 0)
    else:
        if leftSensorValue >= TRACKER_THRESHOLD or rightSensorValue >= TRACKER_THRESHOLD:
            motor.move(BWD, Mid_Speed)
            sleep(0.1)
        elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD:
            counter = counter + 1
            if counter == 3:
                motor.move(FWD, Mid_Speed)
                sleep(0.1)
                counter = 0
            else: 
                motor.move(LEFT, Mid_Speed)
                sleep(0.1)
                motor.move(STOP, 0)
        else:
            motor.move(STOP, 0)
