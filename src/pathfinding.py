import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2

SERVO_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)

def set_servo_angle(angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    duty = angle / 18 + 2
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(0)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)

lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 100, 100])
upper_red2 = np.array([180, 255, 255])

lower_green = np.array([40, 100, 100])
upper_green = np.array([80, 255, 255])

MIN_AREA = 5000

try:
    while True:
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        
        red_area = cv2.countNonZero(mask_red)
        green_area = cv2.countNonZero(mask_green)
        
        if red_area > MIN_AREA and red_area > green_area:
            print("Red detected! Moving servo to 0 degrees.")
            set_servo_angle(0)
        elif green_area > MIN_AREA and green_area > red_area:
            print("Green detected! Moving servo to 90 degrees.")
            set_servo_angle(90)
        else:
            print("No significant color detected. Moving servo to 180 degrees.")
            set_servo_angle(180)
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    servo_pwm.stop()
    GPIO.cleanup()
    picam2.stop()
