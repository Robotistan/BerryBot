from time import sleep
from machine import Pin
from machine import Timer
from machine import UART
from machine import I2C
from machine import PWM
import time, utime
from berrybot import TB6612, NEC_ABC, NEC_16, WS2812, IR_RX
from berrybot import WS2812
import machine
from berrybot import TB6612
import time
from machine import time_pulse_us
from berrybot import HCSR04
import math
from machine import ADC
from berrybot import NEC_16
from berrybot import IR_RX

ledRow = 0
ledArrayBuffer = bytearray(5)
rowPins= [7, 11, 12, 13, 17]
colPins = [3, 2, 16, 19, 18]

Smile1 = bytearray([0x08,0x13,0x10,0x13,0x08])

def setLedMatrix():
    for i in range(0, 5):
        machine.Pin(colPins[i], machine.Pin.OUT)
        machine.Pin(rowPins[i], machine.Pin.OUT)

def drawScreen(buffer):
    for i in range(0, 5):
        ledArrayBuffer[i] = buffer[i]

def setColumns(b):
    machine.Pin(colPins[0]).value((~b >> 0) & 0x01)
    machine.Pin(colPins[1]).value((~b >> 1) & 0x01)
    machine.Pin(colPins[2]).value((~b >> 2) & 0x01)
    machine.Pin(colPins[3]).value((~b >> 3) & 0x01)
    machine.Pin(colPins[4]).value((~b >> 4) & 0x01)

def tick(timer):
    global ledRow, drawing
    drawScreen(drawing)
    if (ledRow == 5):
        ledRow = 0
    setColumns(ledArrayBuffer[ledRow])
    machine.Pin(rowPins[ledRow]).value(1)
    time.sleep_ms(1)
    machine.Pin(rowPins[ledRow]).value(0)
    ledRow = ledRow + 1

ws2812 = WS2812(num_leds = 6, pin_num = 6,brightness = 0.2)
pin_button = machine.Pin(10, machine.Pin.IN)
MOTOR_A1_PIN = 25
MOTOR_A2_PIN = 24
MOTOR_B1_PIN = 22
MOTOR_B2_PIN = 23
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21

motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)

sensor = HCSR04(trigger_pin=8, echo_pin=9, echo_timeout_us=10000)
buzzer = PWM(Pin(14))

import math
ldr_left = ADC(29)
ldr_right = ADC(28)
rightSensor = ADC(Pin(27))
leftSensor = ADC(Pin(26))
THRESHOLD = 50000
Heart2 = bytearray([0x06,0x0F,0x1E,0x0F,0x06])
IR_PIN = 20
ir_data = 0
data_rcvd = False

def ir_callback(data, addr, ctrl):
   global ir_data
   global ir_addr, data_rcvd
   if data > 0:
       ir_data = data
       ir_addr = addr
       print("Data {:02x} Addr {:04x}".format(data, addr))
       data_rcvd = True

ir = NEC_16(Pin(IR_PIN, Pin.IN), ir_callback)

drawing = Smile1
setLedMatrix()
myTimer = Timer(-1)
myTimer.init(period=5, mode=Timer.PERIODIC, callback=tick)
ws2812.pixels_set(0,(255, 0, 0))
ws2812.pixels_show()
ws2812.pixels_set(3,(255, 255, 0))
ws2812.pixels_show()
while True:
    motor.stop()
    sleep(0.25)
    buzzer.duty_u16(0)
    if (pin_button.value()) == (1):
        motor.forward(100 * 650)
        time.sleep((0.3))
        motor.backward(100 * 650)
        time.sleep((0.3))
        motor.left(100 * 650)
        time.sleep((0.3))
        motor.right(100 * 650)
        time.sleep((0.3))
    if (sensor.distance_cm()) <= (5) and (sensor.distance_cm()) >= (0):
        buzzer.freq(500)
        buzzer.duty_u16(2000)
        sleep(0.25)
        buzzer.duty_u16(0)
    if (round(round( ldr_left.read_u16() - 0 ) * ( 100 - 0 ) / ( 65535 - 0 ) + 0)) <= (10):
        ws2812.pixels_set(1,(0, 0, 0))
        ws2812.pixels_show()
    elif (round(round( ldr_right.read_u16() - 0 ) * ( 100 - 0 ) / ( 65535 - 0 ) + 0)) <= (10):
        ws2812.pixels_set(2,(0, 0, 0))
        ws2812.pixels_show()
    else:
        ws2812.pixels_set(1,(51, 204, 0))
        ws2812.pixels_show()
        ws2812.pixels_set(2,(51, 51, 255))
        ws2812.pixels_show()
    if round(round( leftSensor.read_u16() - 0 ) * ( 100 - 0 ) / ( 65535 - 0 ) + 0) < 90:
        ws2812.pixels_set(4,(0, 0, 0))
        ws2812.pixels_show()
        
    elif round(round( rightSensor.read_u16() - 0 ) * ( 100 - 0 ) / ( 65535 - 0 ) + 0) < 90:
        ws2812.pixels_set(5,(0, 0, 0))
        ws2812.pixels_show()
        
    else:
        ws2812.pixels_set(4,(204, 51, 204))
        ws2812.pixels_show()
        ws2812.pixels_set(5,(51, 204, 255))
        ws2812.pixels_show()
        
    if data_rcvd == True:
        data_rcvd = False
        if ir_data == IR_RX.number_ok:
            drawing = Heart2
            setLedMatrix()
            myTimer = Timer(-1)
            myTimer.init(period=5, mode=Timer.PERIODIC, callback=tick)
