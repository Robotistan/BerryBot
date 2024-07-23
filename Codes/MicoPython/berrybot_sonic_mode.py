from time import sleep
from machine import time_pulse_us
from berrybot import HCSR04
from machine import Pin
from berrybot import TB6612
import time


sensor = HCSR04(trigger_pin=8, echo_pin=9, echo_timeout_us=10000)
MOTOR_A1_PIN = 25
MOTOR_A2_PIN = 24 
MOTOR_B1_PIN = 22
MOTOR_B2_PIN = 23
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21

motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)

while True:
    print(sensor.distance_cm())
    if (sensor.distance_cm()) > (20):
        motor.forward(100 * 650)
    else:
        motor.stop()
        time.sleep((0.5))
        motor.backward(100 * 650)
        time.sleep((0.3))
        motor.stop()
        time.sleep((0.2))
        motor.left(100 * 650)
        time.sleep((0.3))
        motor.stop()
        time.sleep((0.2))
   