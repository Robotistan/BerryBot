import machine
import array
import rp2
import utime, time
from utime import sleep_us
from machine import Pin, PWM, Timer, time_pulse_us, UART

##########BLE Library##########
class BLE:
    def __init__(self, uart: UART):
        self.uart = uart
        self.ledWasOn = 0
        self.ledTime = 0
        self.bleConnected = False
        self.rxData = bytes()
        self.BLE_BLINK_INTERVAL = 1000  # LED blink interval in milliseconds
        self.BLE_SERVICE_UUID = "6E400001B5A3F393E0A9E50E24DCCA9E"
        self.BLE_RX_UUID = "6E400002B5A3F393E0A9E50E24DCCA9E"
        self.BLE_TX_UUID = "6E400003B5A3F393E0A9E50E24DCCA9E"
        self.BLE_NAME = "BerryBot"
        
    def configure(self):
        # Configure BLE device
        self._send_command(f'AT+BLENAME={self.BLE_NAME}\r\n')
        self._send_command(f'AT+BLESERUUID={self.BLE_SERVICE_UUID}\r\n')
        self._send_command(f'AT+BLERXUUID={self.BLE_RX_UUID}\r\n')
        self._send_command(f'AT+BLETXUUID={self.BLE_TX_UUID}\r\n')
        self._send_command('AT+SYSIOMAP=1,4\r\n')
        self._send_command('+++\r\n')
        self._send_command('AT+TRANSENTER\r\n')

    def _send_command(self, command: str):
        # Send command to the BLE module
        self.uart.write(command)
        self.read()

    def connect(self):
        currentTime = time.ticks_ms()  # Get the current time in milliseconds
        if not self.bleConnected:  # If BLE is not connected
            if time.ticks_diff(currentTime, self.ledTime) > self.BLE_BLINK_INTERVAL:  # If a certain interval has passed
                self.ledTime = currentTime  # Update the LED blink time
                if self.ledWasOn == 0:  # If the LED is off, turn it on
                    self.uart.write("AT+SYSGPIOWRITE=0,0\r\n")
                    self.ledWasOn = 1  # Set LED status to on
                else:  # If the LED is already on, turn it off
                    self.uart.write("AT+SYSGPIOWRITE=0,1\r\n")
                    self.ledWasOn = 0  # Set LED status to off
        else:  # If BLE is connected, keep the LED on
            self.uart.write("AT+SYSGPIOWRITE=0,0\r\n")
            self.ledWasOn = 1

    def read(self):
        time.sleep(0.5)
        self.rxData = bytes()
        while self.uart.any() > 0:
            self.rxData += self.uart.read(1)
        return self.rxData
            
##########LedMatrix Library##########
class LEDMatrix:
    def __init__(self, row_pins, col_pins):
        """
        LEDMatrix class constructor.
        :param row_pins: List of pins for rows.
        :param col_pins: List of pins for columns.
        """
        self.row_pins = row_pins
        self.col_pins = col_pins
        self.led_row = 0
        self.led_array_buffer = [0] * 5  # A buffer to hold the current pixel states
        self.set_led_matrix()
    
    def draw_screen(self, buffer):
        """
        Draw a given screen buffer onto the LED matrix.
        :param buffer: A list of 5 elements, each element represents the columns for a row.
        """
        for i in range(5):
            self.led_array_buffer[i] = buffer[i]
    
    def set_columns(self, b):
        """
        Set the columns to represent the bit pattern `b`.
        :param b: An integer representing the column pattern.
        """
        machine.Pin(self.col_pins[0]).value((~b >> 0) & 0x01)
        machine.Pin(self.col_pins[1]).value((~b >> 1) & 0x01)
        machine.Pin(self.col_pins[2]).value((~b >> 2) & 0x01)
        machine.Pin(self.col_pins[3]).value((~b >> 3) & 0x01)
        machine.Pin(self.col_pins[4]).value((~b >> 4) & 0x01)

    def set_led_matrix(self):
        """
        Initialize the LED matrix by setting the pins for rows and columns to output.
        """
        for pin in self.row_pins:
            machine.Pin(pin, machine.Pin.OUT)
        for pin in self.col_pins:
            machine.Pin(pin, machine.Pin.OUT)
    
    def tick(self, timer):
        """
        This function updates the LED matrix by switching between rows.
        :param timer: The timer interrupt object.
        """
        if self.led_row == 5:
            self.led_row = 0

        self.set_columns(self.led_array_buffer[self.led_row])
        machine.Pin(self.row_pins[self.led_row]).value(1)
        time.sleep_ms(1)
        machine.Pin(self.row_pins[self.led_row]).value(0)
        self.led_row += 1    
    
##########WS2812 Library##########
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)

def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

class WS2812():
    def __init__(self, num_leds=1, pin_num=6, brightness=0.2):
        self.num_leds = num_leds
        self.pin_num = pin_num
        self.brightness = brightness
        self.ar = array.array("I", [0 for _ in range(self.num_leds)])
        self.sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(self.pin_num))
        self.sm.active(1)
        self.pixel_change_time = time.ticks_ms()
        self.pixel_interval=100
        self.pixel_number = 0
        self.pixel_mode = 0
        self.rgb_value = [[0, 0, 127], [0, 0, 127], [0, 0, 127], [0, 0, 127], [0, 0, 127], [0, 0, 127],[0, 0, 127]]

    def pixels_show(self):
        dimmer_ar = array.array("I", [0 for _ in range(self.num_leds)])
        for i,c in enumerate(self.ar):
            r = int(((c >> 8) & 0xFF) * self.brightness)
            g = int(((c >> 16) & 0xFF) * self.brightness)
            b = int((c & 0xFF) * self.brightness)
            dimmer_ar[i] = (g<<16) + (r<<8) + b
        self.sm.put(dimmer_ar, 8)
        time.sleep_ms(10)

    def pixels_set(self, i, color):
        self.ar[i] = (color[1]<<16) + (color[0]<<8) + color[2]

    def pixels_fill(self, color):
        for i in range(len(self.ar)):
            self.pixels_set(i, color)

    def color_chase(self, color, wait):
        for i in range(self.num_leds):
            self.pixels_set(i, color)
            time.sleep(wait)
            self.pixels_show()
        time.sleep(0.2)
        
    def color_update(self, pix_values):
        for i in range(self.num_leds):
            self.rgb_value[i][0] = pix_values[i][0]
            self.rgb_value[i][1] = pix_values[i][1]
            self.rgb_value[i][2] = pix_values[i][2]
        
    def color_function(self, pix_values):
        self.color_update(pix_values)
        current_time = time.ticks_ms()  # Get the current time in milliseconds
        if time.ticks_diff(current_time, self.pixel_change_time) > self.pixel_interval:  # Check interval
            self.pixel_function()  # Update pixels
            self.pixel_change_time = current_time  # Update last change time
            
    def pixel_function(self):
        if self.pixel_mode == 0:
            self.pixels_set(self.pixel_number, (self.rgb_value[self.pixel_number][0], self.rgb_value[self.pixel_number][1], self.rgb_value[self.pixel_number][2]))
            self.pixel_number += 1
            self.pixels_show()  # Update the strip
            if self.pixel_number > 6:
                self.pixel_number = 0
                self.pixel_mode = 1
        elif self.pixel_mode == 1:
            self.pixel_number += 1
            if self.pixel_number > 6:
                self.pixel_number = 0
                self.pixel_mode = 2
        elif self.pixel_mode == 2:  # Turn off the current pixel by setting its color to (0, 0, 0)
            self.pixels_set(self.pixel_number, (0, 0, 0))
            self.pixel_number += 1
            self.pixels_show()  # Update the strip
            if self.pixel_number > 6:
                self.pixel_number = 0
                self.pixel_mode = 3
        elif self.pixel_mode == 3:
            self.pixel_number += 1
            if self.pixel_number > 6:
                self.pixel_number = 0
                self.pixel_mode = 0
##########IR Reciever Library##########
class IR_RX():
    REPEAT = -1
    # Error codes
    BADSTART = -2
    BADBLOCK = -3
    BADREP = -4
    OVERRUN = -5
    BADDATA = -6
    BADADDR = -7
    
    number_1 = 0x45
    number_2 = 0x46
    number_3 = 0x47
    number_4 = 0x44
    number_5 = 0x40
    number_6 = 0x43
    number_7 = 0x07
    number_8 = 0x15
    number_9 = 0x09
    number_ok = 0x1c
    number_up = 0x18
    number_right = 0x5a
    number_left = 0x08
    number_down = 0x52
    
    def __init__(self, pin, nedges, tblock, callback, *args):
        self._pin = pin
        self._nedges = nedges
        self._tblock = tblock
        self.callback = callback
        self.args = args
        self._errf = lambda _ : None
        self.verbose = False

        self._times = array.array('i',  (0 for _ in range(nedges + 1))) 
        pin.irq(handler = self._cb_pin, trigger = (Pin.IRQ_FALLING | Pin.IRQ_RISING))
        self.edge = 0
        self.tim = Timer(-1) 
        self.cb = self.decode

    def _cb_pin(self, line):
        t = utime.ticks_us()
        if self.edge <= self._nedges:  
            if not self.edge:  
                self.tim.init(period=self._tblock , mode=Timer.ONE_SHOT, callback=self.cb)
            self._times[self.edge] = t
            self.edge += 1
        if self.edge > 68 :
            self.edge = 0

    def do_callback(self, cmd, addr, ext, thresh=0):
        self.edge = 0
        if cmd >= thresh:
            self.callback(cmd, addr, ext, *self.args)
        else:
            self._errf(cmd)

    def error_function(self, func):
        self._errf = func

    def close(self):
        self._pin.irq(handler = None)
        self.tim.deinit()

class NEC_ABC(IR_RX):
    def __init__(self, pin, extended, callback, *args):
        super().__init__(pin, 68, 80, callback, *args)
        self._extended = extended
        self._addr = 0

    def decode(self, _):
        try:
            if self.edge > 68:
                raise RuntimeError(self.OVERRUN)
            width = utime.ticks_diff(self._times[1], self._times[0])
            if width < 4000: 
                raise RuntimeError(self.BADSTART)
            width = utime.ticks_diff(self._times[2], self._times[1])
            if width > 3000: 
                if self.edge < 68: 
                    raise RuntimeError(self.BADBLOCK)
                val = 0
                for edge in range(3, 68 - 2, 2):
                    val >>= 1
                    if utime.ticks_diff(self._times[edge + 1], self._times[edge]) > 1120:
                        val |= 0x80000000
            elif width > 1700:
                raise RuntimeError(self.REPEAT if self.edge == 4 else self.BADREP)  
            else:
                raise RuntimeError(self.BADSTART)
            addr = val & 0xff  # 8 bit addr
            cmd = (val >> 16) & 0xff
            if cmd != (val >> 24) ^ 0xff:
                raise RuntimeError(self.BADDATA)
            if addr != ((val >> 8) ^ 0xff) & 0xff: 
                if not self._extended:
                    raise RuntimeError(self.BADADDR)
                addr |= val & 0xff00 
            self._addr = addr
        except RuntimeError as e:
            cmd = e.args[0]
            addr = self._addr if cmd == self.REPEAT else 0  
        self.do_callback(cmd, addr, 0, self.REPEAT)

class NEC_8(NEC_ABC):
    def __init__(self, pin, callback, *args):
        super().__init__(pin, False, callback, *args)

class NEC_16(NEC_ABC):
    def __init__(self, pin, callback, *args):
        super().__init__(pin, True, callback, *args)
        
##########TB6612 Library##########
FWD = 1
BWD = 2
RIGHT = 3
LEFT = 4
STOP = 0

class TB6612():
    def __init__(self, AIN1, AIN2, BIN1, BIN2, PWMA, PWMB):
        self.MOTOR_A1 = Pin(AIN1, Pin.OUT)
        self.MOTOR_A2 = Pin(AIN2, Pin.OUT)
        self.MOTOR_B1 = Pin(BIN1, Pin.OUT)
        self.MOTOR_B2 = Pin(BIN2, Pin.OUT)
        self.PWM_A = PWM(Pin(PWMA))
        self.PWM_B = PWM(Pin(PWMB))
        self.PWM_A.freq(1000)
        self.PWM_B.freq(1000)
        self.PWM_A.duty_u16(0)
        self.PWM_B.duty_u16(0)
        
    def setMotorSpeed(self, left_speed, right_speed):
        if right_speed > 0:
            self.MOTOR_A1.value(0)  # Set MOTOR_A1 to LOW
            self.MOTOR_A2.value(1)  # Set MOTOR_A2 to HIGH
            if right_speed < 100:
                right_speed = 100
            self.PWM_A.duty_u16(int(right_speed * 257))  # Set the PWM duty cycle for right motor
        else:
            self.MOTOR_A1.value(1)  # Set MOTOR_A1 to HIGH
            self.MOTOR_A2.value(0)  # Set MOTOR_A2 to LOW
            if -right_speed < 100:
                right_speed = -100
            self.PWM_A.duty_u16(int(abs(right_speed) * 257))  # Set the PWM duty cycle for right motor

        if left_speed > 0:
            self.MOTOR_B1.value(0)  # Set MOTOR_B1 to LOW
            self.MOTOR_B2.value(1)  # Set MOTOR_B2 to HIGH
            if left_speed < 100:
                left_speed = 100
            self.PWM_B.duty_u16(int(left_speed * 257))  # Set the PWM duty cycle for left motor
        else:
            self.MOTOR_B1.value(1)  # Set MOTOR_B1 to HIGH
            self.MOTOR_B2.value(0)  # Set MOTOR_B2 to LOW
            if -left_speed < 100:
                left_speed = -100
            self.PWM_B.duty_u16(int(abs(left_speed) * 257))  # Set the PWM duty cycle for left motor
        
    def move(self, direction, speed):
        if direction == FWD:  # Forward
            self.PWM_A.duty_u16(speed)
            self.PWM_B.duty_u16(speed)

            self.MOTOR_A1.value(1)
            self.MOTOR_A2.value(0)
            self.MOTOR_B1.value(1)
            self.MOTOR_B2.value(0)
        elif direction == BWD:  # Backward
            self.PWM_A.duty_u16(speed)
            self.PWM_B.duty_u16(speed)

            self.MOTOR_A1.value(0)
            self.MOTOR_A2.value(1)
            self.MOTOR_B1.value(0)
            self.MOTOR_B2.value(1)
        elif direction == RIGHT:  # Right
            self.PWM_A.duty_u16(speed)
            self.PWM_B.duty_u16(speed)

            self.MOTOR_A1.value(1)
            self.MOTOR_A2.value(0)
            self.MOTOR_B1.value(0)
            self.MOTOR_B2.value(1)
        elif direction == LEFT:  # Left
            self.PWM_A.duty_u16(speed)
            self.PWM_B.duty_u16(speed)

            self.MOTOR_A1.value(0)
            self.MOTOR_A2.value(1)
            self.MOTOR_B1.value(1)
            self.MOTOR_B2.value(0)
        elif direction == STOP:  # Stop
            self.PWM_A.duty_u16(0)
            self.PWM_B.duty_u16(0)

            self.MOTOR_A1.value(0)
            self.MOTOR_A2.value(0)
            self.MOTOR_B1.value(0)
            self.MOTOR_B2.value(0)
        
##########LedMatrix Library##########
class LedMatrix():
    def __init__(self, row_pin_one, row_pin_two, row_pin_three, row_pin_four, row_pin_five, col_pin_one, col_pin_two, col_pin_three, col_pin_four, col_pin_five):
        self.row_one = row_pin_one
        self.row_two = row_pin_two
        self.row_three = row_pin_three
        self.row_four = row_pin_four
        self.row_five = row_pin_five
        
        self.col_one = col_pin_one
        self.col_two = col_pin_two
        self.col_three = col_pin_three
        self.col_four = col_pin_four
        self.col_five = col_pin_five
        
    def drawScreen(self, *buffer):
        for i in range(0,5):
            self.setColumns(buffer[i])
            self.rowPins[i].value(1)
            time.sleep(0.5)
            self.rowPins[i].value(0)
            
    def setColumns(self, b):  
        self.col_one.value((~b >> 0) & 0x01)
        self.col_two.value((~b >> 1) & 0x01)
        self.col_three.value((~b >> 2) & 0x01)
        self.col_four.value((~b >> 3) & 0x01)
        self.col_five.value((~b >> 4) & 0x01)
            
##########HCSR04 Library##########          
class HCSR04:
    def __init__(self, trigger_pin, echo_pin, echo_timeout_us=500*2*30):
        self.echo_timeout_us = echo_timeout_us
        self.trigger = Pin(trigger_pin, mode=Pin.OUT, pull=None)
        self.trigger.value(0)
        self.echo = Pin(echo_pin, mode=Pin.IN, pull=None)

    def _send_pulse_and_wait(self):
        self.trigger.value(0) 
        sleep_us(5)
        self.trigger.value(1)
        sleep_us(10)
        self.trigger.value(0)
        try:
            pulse_time = time_pulse_us(self.echo, 1, self.echo_timeout_us)
            if pulse_time < 0:
                MAX_RANGE_IN_CM = const(500) 
                pulse_time = int(MAX_RANGE_IN_CM * 29.1) 
            return pulse_time
        except OSError as ex:
            if ex.args[0] == 110:
                raise OSError('Out of range')
            raise ex

    def distance_cm(self):
        pulse_time = self._send_pulse_and_wait()
        cms = (pulse_time / 2) / 29.1
        return cms
