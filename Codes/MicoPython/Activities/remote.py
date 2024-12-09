##########Libraries##########
from machine import Pin, PWM, Timer, ADC
import time, utime
from berrybot import TB6612, NEC_ABC, NEC_16, IR_RX
##########Pin Defination##########
TX_PIN = 0
RX_PIN = 1
MOTOR_A1_PIN = 25
MOTOR_A2_PIN = 24
MOTOR_B1_PIN = 22
MOTOR_B2_PIN = 23
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21
IR_PIN = 20
#############Variables####################
ir_data = 0
data_rcvd = False
##########Function Declaration##########
def ir_callback(data, addr, ctrl):
    global ir_data
    global ir_addr, data_rcvd
    if data > 0:
        ir_data = data
        ir_addr = addr
        print('Data {:02x} Addr {:04x}'.format(data, addr))
        data_rcvd = True
        
ir = NEC_16(Pin(IR_PIN, Pin.IN), ir_callback)
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)

while True:
    motor.stop()
    if data_rcvd == True:
        data_rcvd = False
        if ir_data == IR_RX.number_up:          
            motor.forward(65535)
            time.sleep_ms(500)
        elif ir_data == IR_RX.number_down:
            motor.backward(65535)
            time.sleep_ms(500)
        elif ir_data == IR_RX.number_left:
            motor.left(65535)
            time.sleep_ms(130)
        elif ir_data == IR_RX.number_right:
            motor.right(65535)
            time.sleep_ms(130)
        else:
            motor.stop()
