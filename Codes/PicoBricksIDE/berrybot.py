import machine
import array
import rp2
import utime, time
from utime import sleep_us
from machine import Pin, PWM, Timer, time_pulse_us

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
class TB6612():
    def __init__(self, AIN1, AIN2, BIN1, BIN2, PWMA, PWMB):
        self.A1 = Pin(AIN1, Pin.OUT)
        self.A2 = Pin(AIN2, Pin.OUT)
        self.B1 = Pin(BIN1, Pin.OUT)
        self.B2 = Pin(BIN2, Pin.OUT)
        self.apwm = PWM(Pin(PWMA))
        self.bpwm = PWM(Pin(PWMB))
        self.apwm.freq(60)
        self.bpwm.freq(60)
        self.apwm.duty_u16(0)
        self.bpwm.duty_u16(0)
        
    def forward(self, speed):
        self.A1.value(1)
        self.A2.value(0)
        self.B1.value(1)
        self.B2.value(0)

        self.apwm.duty_u16(speed)
        self.bpwm.duty_u16(speed)

    def backward(self, speed):  
        self.A1.value(0)
        self.A2.value(1)
        
        self.B1.value(0)
        self.B2.value(1)
        
        self.apwm.duty_u16(speed)
        self.bpwm.duty_u16(speed)
        
    def right(self, speed):   
        self.A1.value(1)
        self.A2.value(0)
        
        self.B1.value(0)
        self.B2.value(1)
        
        self.apwm.duty_u16(speed)
        self.bpwm.duty_u16(speed)
        
    def left(self, speed):  
        self.A1.value(0)
        self.A2.value(1)
        
        self.B1.value(1)
        self.B2.value(0)
        
        self.apwm.duty_u16(speed)
        self.bpwm.duty_u16(speed)

    def stop(self):   
        self.A1.value(0)
        self.A2.value(0)
        
        self.B1.value(0)
        self.B2.value(0)
        
        self.apwm.duty_u16(0)
        self.bpwm.duty_u16(0)
        
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
