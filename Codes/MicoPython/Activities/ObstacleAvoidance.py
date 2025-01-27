##########Libraries##########
from machine import Pin,
from time import sleep
from berrybot import HCSR04, TB6612
import time
##########Pin Defination##########
MOTOR_A1_PIN = 25
MOTOR_A2_PIN = 24 
MOTOR_B1_PIN = 22
MOTOR_B2_PIN = 23
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21
MODE_BUTTON = 10

Max_Speed = 65000
Mid_Speed = 50000
Low_Speed = 47000
##########Pin Initialization##########
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
sensor = HCSR04(trigger_pin=8, echo_pin=9, echo_timeout_us=10000)
push_button = Pin(MODE_BUTTON,Pin.IN,Pin.PULL_DOWN)
counter=0
right_counter=0
left_counter=0

while (push_button.value()==0):
    motor.stop()

while True:
    print(sensor.distance_cm())
   
    if (sensor.distance_cm()) > (12):
        motor.move(FWD, Max_Speed)
    else:
        motor.move(STOP, 0)
        time.sleep((0.5))
        motor.move(BWD, Max_Speed)
        time.sleep((0.1))
        motor.move(STOP, 0)
        time.sleep((0.2))
        motor.move(LEFT, Mid_Speed)
        if left_counter==0:
            time.sleep((0.5))
            left_counter=left_counter+1
        else:
            time.sleep((1))
            left_counter=0
        motor.move(STOP, 0)
        time.sleep((0.5))
