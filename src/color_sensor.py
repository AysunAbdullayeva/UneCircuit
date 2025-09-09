# -*- coding: utf-8 -*-
from gpiozero import DigitalOutputDevice, DigitalInputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
import time
import statistics

factory = PiGPIOFactory()

S0 = DigitalOutputDevice(17, pin_factory=factory)
S1 = DigitalOutputDevice(27, pin_factory=factory)
S2 = DigitalOutputDevice(22, pin_factory=factory)
S3 = DigitalOutputDevice(23, pin_factory=factory)
OUT = DigitalInputDevice(24, pin_factory=factory)
LED = DigitalOutputDevice(25, pin_factory=factory)

LED.on()

S0.off()
S1.on()

def read_period(s2_val, s3_val):
    S2.value = s2_val
    S3.value = s3_val
    while OUT.value == 1:
        pass
    start = time.time()
    while OUT.value == 0:
        pass
    end = time.time()
    return end - start

def get_color(averages=3):
    red_periods = []
    green_periods = []
    blue_periods = []
    
    for _ in range(averages):
        red_periods.append(read_period(0, 0))
        green_periods.append(read_period(1, 1))
        blue_periods.append(read_period(0, 1))
    
    red = statistics.mean(red_periods)
    green = statistics.mean(green_periods)
    blue = statistics.mean(blue_periods)
    
    return red, green, blue

def periods_to_rgb(red_p, green_p, blue_p):
    if red_p == 0 or green_p == 0 or blue_p == 0:
        return 0, 0, 0
    
    i_red = 1 / red_p
    i_green = 1 / green_p
    i_blue = 1 / blue_p
    
    max_i = max(i_red, i_green, i_blue)
    if max_i == 0:
        return 0, 0, 0
    
    r = int(255 * (i_red / max_i))
    g = int(255 * (i_green / max_i))
    b = int(255 * (i_blue / max_i))
    
    return r, g, b

def is_target_blue(r, g, b):
    return b >= 40 and b <= 255 and r >= 0 and r <= 160 and g >= 0 and g <= b and b > r and b > g

try:
    print("Reng sensoru testi baslayir. Mavi xette yaxinlasdirin (2 sm mesafede).")
    blue_count = 0
    on_blue = False
    while blue_count < 12:
        periods = get_color(averages=3)
        r_period, g_period, b_period = periods
        print(f"Raw periods: R: {r_period}, G: {g_period}, B: {b_period}")
        print(f"Periods (s): R: {r_period*1e6:.2f}, G: {g_period*1e6:.2f}, B: {b_period*1e6:.2f}")
        
        r, g, b = periods_to_rgb(r_period, g_period, b_period)
        print(f"RGB: R={r}, G={g}, B={b}")
        
        if is_target_blue(r, g, b):
            if not on_blue:
                blue_count += 1
                on_blue = True
                print(f"Mavi xett askarlandi! Say: {blue_count}")
            else:
                print("Mavi xett davam edir")
        else:
            on_blue = False
            print("Mavi xett askarlanmadi. (Ferqli seth ve ya isiq seraity?)")
        
        time.sleep(0.1)
    
    print("Proqram bitdi")
    time.sleep(2.5)
    LED.off()
    S0.close()
    S1.close()
    S2.close()
    S3.close()
    OUT.close()
    LED.close()

except KeyboardInterrupt:
    print("Proqram dayandirildi")
    LED.off()
    S0.close()
    S1.close()
    S2.close()
    S3.close()
    OUT.close()
    LED.close()
