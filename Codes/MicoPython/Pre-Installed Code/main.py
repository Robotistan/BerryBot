##########Libraries##########
from machine import I2C, Pin, PWM, UART, Timer, ADC
import time, utime
from time import sleep
import random
from berrybot import TB6612, NEC_ABC, NEC_16, WS2812, IR_RX, BLE, LEDMatrix, HCSR04
##########Pin Defination##########
TX_PIN = 0
RX_PIN = 1
BUZZER_PIN = 14
NEOPIXEL_PIN = 6
MOTOR_A1_PIN = 24
MOTOR_A2_PIN = 25
MOTOR_B1_PIN = 23
MOTOR_B2_PIN = 22
MOTOR_PWM_A_PIN = 15
MOTOR_PWM_B_PIN = 21
LDR_L_PIN = 29
LDR_R_PIN = 28
LEFT_TRACKER = 26
RIGHT_TRACKER = 27
IR_PIN = 20
ECHO_PIN = 9
TRIG_PIN = 8
rowPins= [7, 11, 12, 13, 17]
colPins = [18, 19, 16, 2, 3]
rgb_value = [[0, 0, 127], [0, 0, 127], [0, 0, 127], [0, 0, 127], [0, 0, 127], [0, 0, 127],[0, 0, 127]]
#############Variables####################
STOP =  0
FWD =   1
BWD =   2
RIGHT = 3
LEFT =  4

Max_Speed = 65000
Mid_Speed = 50000
Low_Speed = 47000

TRACKER_THRESHOLD = 50000
LDR_THRESHOLD = 250
LDR_TOLERANCE = 5000

ir_data = 0
data_rcvd = False
ledRow = 0
ledArrayBuffer = bytearray(5)
user_led_matrix = bytearray(5)
ble_buf = bytes()
berryMode = 0
lastButtonState = 0
led_matrix_status = 0
rgb_status = 1
bluetooth_mode = 0
directionStt = STOP
oldDirection = STOP
leftSensorValue = 0
rightSensorValue = 0
counter=0
right_counter=0
left_counter=0
distance = 0
#############Led Matrix###################
smile =     [0x0A,0x0A,0x00,0x11,0x0E]
yes =       [0x00,0x01,0x02,0x14,0x08]
no =        [0x11,0x0A,0x04,0x0A,0x11]
left =      [0x04,0x0E,0x15,0x04,0x04]
right =     [0x04,0x04,0x15,0x0E,0x04]
forward =   [0x04,0x02,0x1F,0x02,0x04]
backward =  [0x04,0x08,0x1F,0x08,0x04]
empty =     [0x00,0x00,0x00,0x00,0x00]
full =      [0x1F,0x1F,0x1F,0x1F,0x1F]
heart =     [0x0A,0x1F,0x1F,0x0E,0x04]
sad =       [0x00,0x0A,0x00,0x0E,0x11]
tracker =   [0x07,0x04,0x1F,0x04,0x07]
bluetooth = [0x1F,0x00,0x0E,0x00,0x04]
ir_img =    [0x1F,0x11,0x1F,0x11,0x1F]
sunny =     [0x15,0x0E,0x1F,0x0E,0x15]
sonic =     [0x04,0x0E,0x1B,0x0E,0x04]
triangle =  [0x1F,0x11,0x11,0x0A,0x04]
##########Pin Initialization##########
uart = UART(0, 115200, parity=None, stop = 1, bits = 8, tx=Pin(0), rx=Pin(1),timeout=10)
ble = BLE(uart)
ble.configure()
motor = TB6612(MOTOR_A1_PIN, MOTOR_A2_PIN, MOTOR_B1_PIN, MOTOR_B2_PIN, MOTOR_PWM_A_PIN, MOTOR_PWM_B_PIN)
sensor = HCSR04(trigger_pin=8, echo_pin=9, echo_timeout_us=10000)
button = Pin(10, Pin.IN)
rgb = WS2812(7,NEOPIXEL_PIN,0.2)
matrix = LEDMatrix(rowPins, colPins)
ldr_left = ADC(LDR_L_PIN)
ldr_right = ADC(LDR_R_PIN)
leftSensor = ADC(Pin(LEFT_TRACKER))
rightSensor = ADC(Pin(RIGHT_TRACKER))
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.freq(1000) # Buzzer frequency set to 1 kHz
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
    
def berry_horn():
    buzzer.duty_u16(32768)
    time.sleep(0.3)
    buzzer.duty_u16(0)
    
def buttonInterruptHandler(event):    # Interrupt event, that will work when button is pressed
    global berryMode, lastButtonState
    motor.move(STOP,0)
    buttonState = button.value()
    if buttonState == 1 and lastButtonState == 0:
        lastButtonState = 1
        if berryMode >= 5:
            berryMode = 0
        else:
            berryMode += 1
        time.sleep(0.5)
    elif buttonState == 0 and lastButtonState == 1:
        lastButtonState = 0
        time.sleep(0.5)
        
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def constrain(x, min_val, max_val):
    return max(min(x, max_val), min_val)

def drawScreen(buffer):
    led_array_buffer = [0] * 5  # Initialize the LED array buffer with 5 elements.
    for i in range(5):
        led_array_buffer[i] = buffer[i]
    return led_array_buffer  # Return the updated LED array buffer.

##########Modes##########
def remote():
    global data_rcvd, ir_data
    motor.move(STOP,0)
    if data_rcvd == True:
        data_rcvd = False
        if ir_data == IR_RX.number_up: # Forward
            motor.move(FWD, Max_Speed)
            time.sleep_ms(500)
        elif ir_data == IR_RX.number_down: # Backward
            motor.move(BWD, Max_Speed)
            time.sleep_ms(500)
        elif ir_data == IR_RX.number_left: #Left
            motor.move(LEFT, Max_Speed)
            time.sleep_ms(130)
        elif ir_data == IR_RX.number_right: # Right
            motor.move(RIGHT, Max_Speed)
            time.sleep_ms(130)
        else:
            motor.move(STOP,0)
            
def lineTracker():
    global leftSensorValue, rightSensorValue, directionStt, oldDirection
    leftSensorValue = leftSensor.read_u16()
    rightSensorValue = rightSensor.read_u16()
    time.sleep(0.02)
    
    if leftSensorValue >= TRACKER_THRESHOLD and rightSensorValue >= TRACKER_THRESHOLD:
        directionStt = FWD
    elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue > TRACKER_THRESHOLD:
        directionStt = RIGHT
    elif leftSensorValue > TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD:
        directionStt = LEFT
    elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD and directionStt != STOP:
        directionStt = BWD
        
    if directionStt != oldDirection:
        oldDirection = directionStt
        if directionStt == FWD:
            motor.move(FWD, Mid_Speed)
        elif directionStt == RIGHT:
            motor.move(RIGHT, Mid_Speed)
        elif directionStt == LEFT:
            motor.move(LEFT, Mid_Speed)
        elif directionStt == BWD:
            motor.move(BWD, Low_Speed)
            
def lightTracker():
    LDR_L = ldr_left.read_u16()
    LDR_R = ldr_right.read_u16()
    sleep(0.02)
    distance = sensor.distance_cm()
    sleep(0.02)
    
    if LDR_L >= LDR_THRESHOLD and LDR_R >= LDR_THRESHOLD:
        if distance < 10:
            motor.move(STOP,0)
            sleep(0.02)
            motor.move(LEFT, Mid_Speed)
            sleep(0.5)
            motor.move(STOP, 0)
        elif (LDR_R - LDR_L) >= LDR_TOLERANCE:
            motor.move(RIGHT, Mid_Speed)
        elif (LDR_L - LDR_R) >= LDR_TOLERANCE:
            motor.move(LEFT, Mid_Speed)   
        elif ( LDR_L >= 10000 and LDR_R >= 10000):
            motor.move(FWD, Max_Speed)
        else:
            motor.move(STOP, 0)
    else:
        motor.move(STOP, 0)
        
def sumo():
    global distance, leftSensorValue, rightSensorValue, counter
    
    distance = sensor.distance_cm()
    leftSensorValue = leftSensor.read_u16()
    rightSensorValue = rightSensor.read_u16()
    sleep(0.02)
    if distance <= 15:
        if leftSensorValue >= TRACKER_THRESHOLD or rightSensorValue >= TRACKER_THRESHOLD:
            motor.move(BWD, Mid_Speed)
            sleep(0.5)
        elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD:
            motor.move(FWD, Mid_Speed)
            sleep(0.5)
        else:
            motor.move(STOP, 0)
    else:
        if leftSensorValue >= TRACKER_THRESHOLD or rightSensorValue >= TRACKER_THRESHOLD:
            motor.move(BWD, Mid_Speed)
            sleep(0.1)
        elif leftSensorValue < TRACKER_THRESHOLD and rightSensorValue < TRACKER_THRESHOLD:
            counter = counter + 1
            if counter == 3:
                motor.move(FWD, Mid_Speed)
                sleep(0.1)
                counter = 0
            else: 
                motor.move(LEFT, Mid_Speed)
                sleep(0.1)
                motor.move(STOP, 0)
        else:
            motor.move(STOP, 0)
            
def sonic():
    global counter, right_counter, left_counter
    
    if (sensor.distance_cm()) > (12):
        motor.move(FWD, Max_Speed)
    else:
        motor.move(STOP, 0)
        time.sleep((0.5))
        motor.move(BWD, Max_Speed)
        time.sleep((0.1))
        motor.move(STOP, 0)
        time.sleep((0.2))
        motor.move(LEFT, Mid_Speed)
        if left_counter==0:
            time.sleep((0.5))
            left_counter=left_counter+1
        else:
            time.sleep((1))
            left_counter=0
        motor.move(STOP, 0)
        time.sleep((0.5))
#########################################
ir = NEC_16(Pin(IR_PIN, Pin.IN), ir_callback)
button.irq(trigger=Pin.IRQ_RISING, handler=buttonInterruptHandler)
matrix.set_led_matrix()
myTimer = Timer(-1)
myTimer.init(period=5, mode=Timer.PERIODIC, callback=matrix.tick)

start_time = time.ticks_ms()  # Başlangıç zamanını al
time.sleep(1)  # 1 saniye bekle
elapsed_time = time.ticks_diff(time.ticks_ms(), start_time)


while True:
    lastButtonState = 0
    if rgb_status == 1:
        rgb.color_function(rgb_value)

    if berryMode == 0:  #Bluetooth Mode
        if(led_matrix_status == 0):
            matrix.draw_screen(bluetooth)
        elif (led_matrix_status == 1):
            matrix.draw_screen(user_led_matrix)
            
        if(rgb_status == 1):
            rgb.color_function(rgb_value)
            
        ble_buf = ble.read()
        if ble_buf != b'':
            for i in range(len(ble_buf)):
                print(ble_buf[i])
            if ((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 99)): #Exit modes
                bluetooth_mode = 0
                motor.move(STOP,0)
            elif((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 1) and (ble_buf[3] == 0)): # Neo turn off
                bluetooth_mode = 0
                rgb_status = 0
                for i in range(6):
                    rgb.pixels_set(i, (0, 0, 0))
                    rgb.pixels_show()
                    time.sleep(0.05)
            elif ((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 90) and (ble_buf[3] == 0)): # Neo turn on
                bluetooth_mode = 0
                rgb_status = 1
            elif ((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 2) and (ble_buf[3] == 0)): # Horn
                bluetooth_mode = 0
                berry_horn()
            elif (((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 4) and (ble_buf[3] == 0)) or (bluetooth_mode == 1)): # Sonic
                bluetooth_mode = 1
                sonic()
            elif (((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 8) and (ble_buf[3] == 0)) or (bluetooth_mode == 2)): # Line Tracker
                bluetooth_mode = 2
                lineTracker()
            elif (((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 16) and (ble_buf[3] == 0)) or (bluetooth_mode == 3)): # Light Tracker
                bluetooth_mode = 3
                lightTracker()
            elif (((ble_buf[0] == 82) and (ble_buf[1] == 2) and (ble_buf[2] == 32) and (ble_buf[3] == 0)) or (bluetooth_mode == 4)): # Sumo
                bluetooth_mode = 4
                sumo()
            elif ((ble_buf[0] == 82) and (ble_buf[1] == 3) and (ble_buf[2] != 255) and (ble_buf[3] != 255)): # Move
                bluetooth_mode = 0
                led_matrix_status = 2
                
                if((ble_buf[2] == 0) and (ble_buf[3] == 0)):
                    led_matrix_status = 0
                    motor.move(STOP,0)
                    matrix.draw_screen(bluetooth)
                else:
                    joyX = ble_buf[2]
                    joyY = ble_buf[3]

                    mappedX = map_value(joyX, 0, 255, -255, 255)
                    mappedY = map_value(joyY, 0, 255, -255, 255)
                    leftMotorSpeed = mappedY + mappedX
                    rightMotorSpeed = mappedY - mappedX

                    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255)
                    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255)
                    
                    motor.setMotorSpeed(leftMotorSpeed, rightMotorSpeed)
                
            elif ((ble_buf[0] == 82) and (ble_buf[1] == 7)): # RGB Menu
                bluetooth_mode = 0
                
                rgb_value[ble_buf[2]-1][0] = ble_buf[3]
                rgb_value[ble_buf[2]-1][1] = ble_buf[4]
                rgb_value[ble_buf[2]-1][2] = ble_buf[5]
            elif ((ble_buf[0] == 82) and (ble_buf[1] == 6) and (ble_buf[2] != 255) and (ble_buf[3] != 255) and (ble_buf[4] != 255) and (ble_buf[5] != 255) and (ble_buf[6] != 255)): #Led Matrix
                bluetooth_mode = 0
                
                user_led_matrix[0] = ble_buf[2]
                user_led_matrix[1] = ble_buf[3]
                user_led_matrix[2] = ble_buf[4]
                user_led_matrix[3] = ble_buf[5]
                user_led_matrix[4] = ble_buf[6]

                led_matrix_status = 1
                drawScreen(user_led_matrix)
                
            ble_buf = bytes()
            #time.sleep(0.05)
        else:
            pass
    if berryMode == 1:  #IR Mode
        rgb.color_function(rgb_value)
        matrix.draw_screen(ir_img)
        remote()
    if berryMode == 2:  #Line Tracker Mode
        rgb.color_function(rgb_value)
        matrix.draw_screen(tracker)
        lineTracker()
    if berryMode == 3:  #Light Tracker Mode
        rgb.color_function(rgb_value)
        matrix.draw_screen(sunny)
        lightTracker()
    if berryMode == 4:  #Sonic Mode
        rgb.color_function(rgb_value)
        matrix.draw_screen(sonic);
        sonic()
    if berryMode == 5:  #Sumo Mode
        rgb.color_function(rgb_value)
        matrix.draw_screen(triangle)
        sumo()
