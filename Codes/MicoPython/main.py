##########Libraries##########
from machine import I2C, Pin, PWM, UART, Timer, ADC
import time, utime
import random
from berrybot import TB6612, NEC_ABC, NEC_16, WS2812, IR_RX
##########Pin Defination##########
TX_PIN = 0
RX_PIN = 1
BUZZER_PIN = 14
NEOPIXEL_PIN = 6
MOTOR_A1_PIN = 25
MOTOR_A2_PIN = 24
MOTOR_B1_PIN = 22
MOTOR_B2_PIN = 23
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21
LDR_L_PIN = 29
LDR_R_PIN = 28
IR_PIN = 20
ECHO_PIN = 11
TRIG_PIN = 12
rowPins= [7, 11, 12, 13, 17]
colPins = [3, 2, 16, 19, 18]
#############Variables####################
ir_data = 0
data_rcvd = False
mode = 0
ledRow = 0
ledArrayBuffer = bytearray(5)
#############Led Matrix###################
smile =     [0x08,0x13,0x10,0x13,0x08]
yes =       [0x02,0x04,0x08,0x10,0x08]
no =        [0x11,0x0A,0x04,0x0A,0x11]
left =      [0x04,0x0E,0x15,0x04,0x04]
right =     [0x04,0x04,0x15,0x0E,0x04]
backward =  [0x04,0x02,0x1F,0x02,0x04]
forward =   [0x04,0x08,0x1F,0x08,0x04]
empty =     [0x00,0x00,0x00,0x00,0x00]
full =      [0x1F,0x1F,0x1F,0x1F,0x1F]
sunny =     [0x11,0x0E,0x0A,0x0E,0x11]
heart =     [0x06,0x0F,0x1E,0x0F,0x06]
ghost =     [0x1E,0x0D,0x1F,0x0D,0x1E]
giraffe =   [0x00,0x18,0x08,0x1F,0x01]
duck =      [0x04,0x1E,0x1E,0x18,0x08]
umbrella =  [0x04,0x03,0x1F,0x13,0x12]
sad =       [0x10,0x0A,0x08,0x0A,0x10]
tracker =   [0x07,0x04,0x1F,0x04,0x07]
square =    [0x00,0x0E,0x0A,0x0E,0x00]
diamond =   [0x04,0x0A,0x11,0x0A,0x04]
bluetooth = [0x01,0x05,0x15,0x05,0x01]
##########Pin Initialization##########
#uart = UART(0, 9600, TX_PIN, RX_PIN)
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
ws2812 = WS2812(6,NEOPIXEL_PIN,0.2)
ldr_left = ADC(LDR_L_PIN)
ldr_right = ADC(LDR_R_PIN)
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.duty_u16(0)
##########Function Declaration##########
def ir_callback(data, addr, ctrl):
    global ir_data
    global ir_addr, data_rcvd
    if data > 0:
        ir_data = data
        ir_addr = addr
        print('Data {:02x} Addr {:04x}'.format(data, addr))
        data_rcvd = True
        
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
'''
def tick(timer):    
    ws2812.pixels_fill((0,0,255))
    ws2812.pixels_show()
    time.sleep(0.5)
    ws2812.pixels_fill((0,0,0))
    ws2812.pixels_show()
    time.sleep(0.5)
'''
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

#########################################
ir = NEC_16(Pin(IR_PIN, Pin.IN), ir_callback)
setLedMatrix()
myTimer = Timer(-1)
myTimer.init(period=5, mode=Timer.PERIODIC, callback=tick)

while True:
    ws2812.pixels_fill((0,0,255))
    ws2812.pixels_show()
    time.sleep(0.5)
    ws2812.pixels_fill((0,0,0))
    ws2812.pixels_show()
    time.sleep(0.5)
    
    left_light = ldr_left.read_u16()
    print(left_light)
    
    if data_rcvd == True:
        data_rcvd = False
        if ir_data == IR_RX.number_up:          
            motor.forward(65535)
        if ir_data == IR_RX.number_down:
            motor.backward(65535)
        if ir_data == IR_RX.number_left:
            motor.left(65535)
        if ir_data == IR_RX.number_right:
            motor.right(65535)
        if ir_data == IR_RX.number_ok:
            motor.stop()
        if ir_data == IR_RX.number_1:
            time.sleep(0.5)
            buzzer.duty_u16(2000)
            buzzer.freq(831)
            time.sleep(0.5)
            buzzer.duty_u16(0)

