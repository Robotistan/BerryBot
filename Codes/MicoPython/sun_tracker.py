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
LDR_L_PIN = 29
LDR_R_PIN = 28
rowPins= [7, 11, 12, 13, 17]
colPins = [3, 2, 16, 19, 18]
#############Variables####################
LDR_THRESHOLD = 250
ledRow = 0
ledArrayBuffer = bytearray(5)
#############Led Matrix###################
sunny =     [0x11,0x0E,0x0A,0x0E,0x11]
##########Pin Initialization##########
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
sensor = HCSR04(trigger_pin=12, echo_pin=11, echo_timeout_us=10000)
ldr_left = ADC(LDR_L_PIN)
ldr_right = ADC(LDR_R_PIN)
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
    drawScreen(sunny)
    LDR_L = ldr_left.read_u16()
    LDR_R = ldr_right.read_u16()
    #print(leftSensorValue)
    #print(rightSensorValue)
    sleep(0.02)
    distance = sensor.distance_cm()
    #print(distance)
    sleep(0.02)

    if LDR_L >= LDR_THRESHOLD and LDR_R >= LDR_THRESHOLD:
        if (LDR_R - LDR_L) >= LDR_TOLERANCE:
            motor.right(150)
        elif (LDR_L - LDR_R) >= LDR_TOLERANCE:
            motor.left(150)
        else:
            if distance >= 15
                motor.forward(255)
            else
                motor.stop()
    else:
        motor.stop()
