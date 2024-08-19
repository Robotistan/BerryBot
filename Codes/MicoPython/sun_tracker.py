##########Libraries##########
from time import sleep
from machine import Pin, PWM, UART, Timer, ADC
import time, utime
from berrybot import TB6612, HCSR04
##########Pin Defination##########
MOTOR_A1_PIN = 25
MOTOR_A2_PIN = 24
MOTOR_B1_PIN = 22
MOTOR_B2_PIN = 23
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21
LDR_L_PIN = 29
LDR_R_PIN = 28
MODE_BUTTON = 10
#############Variables####################
LDR_THRESHOLD = 250
LDR_TOLERANCE = 5000

##########Pin Initialization##########
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
sensor = HCSR04(trigger_pin=8, echo_pin=9, echo_timeout_us=10000)
ldr_left = ADC(LDR_L_PIN)
ldr_right = ADC(LDR_R_PIN)
push_button = Pin(MODE_BUTTON,Pin.IN,Pin.PULL_DOWN)
while (push_button.value()==0):
    motor.stop()
    
while True:
   
    LDR_L = ldr_left.read_u16()
    LDR_R = ldr_right.read_u16()
    #print(LDR_L)
    #print(LDR_R)
    sleep(0.02)
    distance = sensor.distance_cm()
    #print(distance)
    sleep(0.02)
    

    if LDR_L >= LDR_THRESHOLD and LDR_R >= LDR_THRESHOLD:
        if distance < 10:
            motor.stop()
            sleep(0.02)
            motor.left(80*650)
            sleep(0.5)
            motor.stop()
        elif (LDR_R - LDR_L) >= LDR_TOLERANCE:
            motor.right(80*650)
        elif (LDR_L - LDR_R) >= LDR_TOLERANCE:
            motor.left(80*650)   
        elif ( LDR_L >= 10000 and LDR_R >= 10000):
            motor.forward(100*650)
        else:
            motor.stop()
    else:
        motor.stop()


