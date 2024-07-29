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
#############Variables####################
MotorSpeed = 50000
threshold = 60000
##########Pin Initialization##########
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
sensor = HCSR04(trigger_pin=12, echo_pin=11, echo_timeout_us=10000)
leftSensor = ADC(Pin(LEFT_TRACKER))
rightSensor = ADC(Pin(RIGHT_TRACKER))

while True:
    distance = sensor.distance_cm()
    #print(distance)
    if distance <= 15:
        leftSensorValue = leftSensor.read_u16()
        rightSensorValue = rightSensor.read_u16()
        #print(leftSensorValue)
        #print(rightSensorValue)
        sleep(0.02)
        if leftSensorValue >= threshold or rightSensorValue >= threshold:
            motor.backward(MotorSpeed)
            sleep(0.5)
        elif leftSensorValue < threshold and rightSensorValue < threshold:
            motor.forward(MotorSpeed)
        else:
            motor.stop()
    else:
        leftSensorValue = leftSensor.read_u16()
        rightSensorValue = rightSensor.read_u16()
        #print(leftSensorValue)
        #print(rightSensorValue)
        sleep(0.02)
        if leftSensorValue >= threshold or rightSensorValue >= threshold:
            motor.backward(MotorSpeed)
            sleep(0.5)
        elif leftSensorValue < threshold and rightSensorValue < threshold:
            motor.left(MotorSpeed)
            sleep(0.1)
            motor.stop()
        else:
            motor.stop()
