from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
import time

factory = PiGPIOFactory()

TRIG_FRONT = 5
ECHO_FRONT = 26
TRIG_LEFT = 12
ECHO_LEFT = 13
TRIG_RIGHT = 19
ECHO_RIGHT = 16
SERVO_PIN = 18

front_sensor = DistanceSensor(echo=ECHO_FRONT, trigger=TRIG_FRONT, pin_factory=factory, max_distance=2.0)
left_sensor = DistanceSensor(echo=ECHO_LEFT, trigger=TRIG_LEFT, pin_factory=factory, max_distance=2.0)
right_sensor = DistanceSensor(echo=ECHO_RIGHT, trigger=TRIG_RIGHT, pin_factory=factory, max_distance=2.0)
servo = AngularServo(SERVO_PIN, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025, pin_factory=factory)

servo.angle = 90

def get_distance(sensor):
    try:
        return sensor.distance * 100
    except:
        return 999 

try:
    while True:
        dist_front = get_distance(front_sensor)
        dist_left = get_distance(left_sensor)
        dist_right = get_distance(right_sensor)
        
        print(f"On: {dist_front:.2f} cm, Sol: {dist_left:.2f} cm, Sag: {dist_right:.2f} cm")
        
        if dist_front < 10 and dist_right > 15:
            servo.angle = 180
            print('turning 180')
        elif dist_front < 10 and dist_left > 15:
            servo.angle = 0
            print('turning 0')
        else:
            servo.angle = 90
            print('turning 90')
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Proqram dayandirildi")

finally:
    front_sensor.close()
    left_sensor.close()
    right_sensor.close()
    servo.close()