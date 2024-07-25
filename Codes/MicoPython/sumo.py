##########Libraries##########
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
LEFT_TRACKER = 26
RIGHT_TRACKER = 27
rowPins= [7, 11, 12, 13, 17]
colPins = [3, 2, 16, 19, 18]
#############Variables####################
MotorSpeed = 50000
threshold = 65000
ledRow = 0
ledArrayBuffer = bytearray(5)
#############Led Matrix###################
smile =     [0x08,0x13,0x10,0x13,0x08]
##########Pin Initialization##########
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
sensor = HCSR04(trigger_pin=12, echo_pin=11, echo_timeout_us=10000)
leftSensor = ADC(Pin(LEFT_TRACKER))
rightSensor = ADC(Pin(RIGHT_TRACKER))
##########Function Declaration##########
def drawScreen(buffer):
    for i in range(0, 5):
        ledArrayBuffer[i] = buffer[i]

def setColumns(b):
    machine.Pin(colPins[0]).value((~b >> 0) & 0x01)
    machine.Pin(colPins[1]).value((~b >> 1) & 0x01)
    machine.Pin(colPins[2]).value((~b >> 2) & 0x01)
    machine.Pin(colPins[3]).value((~b >> 3) & 0x01)
    machine.Pin(colPins[4]).value((~b >> 4) & 0x01)

def setLedMatrix():
    for i in range(0, 5):
        machine.Pin(colPins[i], machine.Pin.OUT)
        machine.Pin(rowPins[i], machine.Pin.OUT)

def tick(timer):
    global ledRow
    drawScreen(smile)
    if (ledRow == 5):
        ledRow = 0

    setColumns(ledArrayBuffer[ledRow])
    machine.Pin(rowPins[ledRow]).value(1)
    time.sleep_ms(1)
    machine.Pin(rowPins[ledRow]).value(0)
    ledRow = ledRow + 1

setLedMatrix()
myTimer = Timer(-1)
myTimer.init(period=5, mode=Timer.PERIODIC, callback=tick)

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
