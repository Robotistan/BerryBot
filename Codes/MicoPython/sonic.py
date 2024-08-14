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
    if (sensor.distance_cm()) > (10):
        motor.forward(100 * 650)
    else:
        motor.stop()
        time.sleep((0.5))
        motor.backward(100 * 650)
        time.sleep((0.05))
        motor.stop()
        time.sleep((0.2))  
        if(right_counter<2):
            motor.right(100 * 650)
            if(counter==0):
                time.sleep((0.25))
                counter=1
            else:
                time.sleep((0.3))
            right_counter=1+right_counter
        else:
            motor.left(100 * 650)
            time.sleep((0.4))
        print(right_counter)
        motor.stop()
        time.sleep((0.2))
   
