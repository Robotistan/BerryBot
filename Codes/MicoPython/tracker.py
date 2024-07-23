##########Libraries##########
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
rowPins= [7, 11, 12, 13, 17]
colPins = [3, 2, 16, 19, 18]
#############Variables####################
Mid_Speed = 50000
Low_Speed = 47000

STOP = 0
FWD = 1
LEFT = 2
RIGHT = 3
BWD = 4

threshold = 65000
directionStt = STOP
oldDirection = STOP
ledRow = 0
ledArrayBuffer = bytearray(5)
#############Led Matrix###################
tracker =   [0x07,0x04,0x1F,0x04,0x07]
##########Pin Initialization##########
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
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
    drawScreen(tracker)
    leftSensorValue = leftSensor.read_u16()
    rightSensorValue = rightSensor.read_u16()
    #print(leftSensorValue)
    #print(rightSensorValue)
    sleep(0.02)

    if leftSensorValue >= threshold and rightSensorValue >= threshold:
        directionStt = FWD
    elif leftSensorValue < threshold and rightSensorValue > threshold:
        directionStt = RIGHT
    elif leftSensorValue > threshold and rightSensorValue < threshold:
        directionStt = LEFT
    elif leftSensorValue < threshold and rightSensorValue < threshold and directionStt != STOP:
        directionStt = BWD
        
    if directionStt != oldDirection:
        oldDirection = directionStt
        if directionStt == FWD:
            motor.forward(Mid_Speed)
        elif directionStt == RIGHT:
            motor.right(Mid_Speed)
        elif directionStt == LEFT:
            motor.left(Mid_Speed)
        elif directionStt == BWD:
            motor.backward(Low_Speed)
            reversTime = utime.ticks_ms()
        elif directionStt == STOP:
            motor.stop()
            
        #print(directionStt)
        #print(oldDirection)
    if directionStt == ("BWD") and (utime.ticks_ms()) - reversTime > (300):
        directionStt = "STOP"
