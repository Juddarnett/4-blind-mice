# Import Libraries
from openni import openni2
import platform
import numpy as np
import RPi.GPIO as GPIO
import time

# Set Up GPIO Pins to BCM
GPIO.setmode(GPIO.BCM)

# Assign Variable to Pin Numbers
# PWM Pins
pwm_left = 25
pwm_upper = 24
pwm_lower = 23
pwm_right = 18

# Battery Logic Pins
logic_A = 17
logic_B = 27

# Setup Pins
# PWM Output Pins
GPIO.setup(pwm_left, GPIO.OUT)
GPIO.setup(pwm_upper, GPIO.OUT)
GPIO.setup(pwm_lower, GPIO.OUT)
GPIO.setup(pwm_right, GPIO.OUT)

# Battery Logic Input Pins
GPIO.setup(logic_A, GPIO.IN)
GPIO.setup(logic_B, GPIO.IN)

# Make PWM Signals
# Set Frequency
PWM_left = GPIO.PWM(pwm_left, 100)
PWM_upper = GPIO.PWM(pwm_upper, 100)
PWM_lower = GPIO.PWM(pwm_lower, 100)
PWM_right = GPIO.PWM(pwm_right, 100)

# Set Default Duty-Cycle to 0
PWM_left.start(0)
PWM_upper.start(0)
PWM_lower.start(0)
PWM_right.start(0)

# Threshold of Sensor
max_int = 2500
percent = 0.10

# Range for X (distance from sensor)
x_max = float(max_int)
x_min = float(x_max * percent)

# y = M * x + b
del_x = x_min - x_max
M = 100 / del_x
C = 100 - M * x_min

# Intialize OpenNI
if platform.system() == 'linux':
    openni2.initialze("home/SDK/TERABEE-Linux-Arm-OpenNI2.2/Redist/OpenNI2/Drivers/libmodule-terabee2.so")
else:
    openni2.initialize()

# Connect and Open Device
dev = openni2.Device.open_any()

# Create Depth Stream
depth_stream = dev.create_depth_stream()
depth_stream.start()

# Size of Array
rows = 60
columns = 80

# Main Loop
while True:
    # Collects Data Array from Lidar Sensor
    frame = depth_stream.read_frame()
    frame_data = frame.get_buffer_as_uint16()
    frame_array = np.asarray(frame_data).reshape((rows, columns))

    # Smaller Arrays
    X_left = frame_array[:, 0:int(columns / 3)]
    X_upper = frame_array[0:int(rows / 2), int(columns / 3):int(2 * columns / 3)]
    X_lower = frame_array[int(rows / 2):int(rows + 1), int(columns / 3):int(2 * columns / 3)]
    X_right = frame_array[:, int(2 * columns / 3):int(columns + 1)]

    # Finds Smallest Value in Each Array
    x_left = np.min(X_left)
    x_upper = np.min(X_upper)
    x_lower = np.min(X_lower)
    x_right = np.min(X_right)

    # Define Motor Duty-Cycle Functions
    def left_motor():
        dc_left = M * x_left + C
        if 100 >= dc_left > 0:
            PWM_left.ChangeDutyCycle(dc_left)
            print(dc_left)
        if dc_left >= 100 or dc_left <= 0:
            PWM_left.ChangeDutyCycle(0)
            print(0)

    def upper_motor():
        dc_upper = M * x_upper + C
        if 100 >= dc_upper > 0:
            PWM_upper.ChangeDutyCycle(dc_upper)
            print(dc_upper)
        if dc_upper >= 100 or dc_upper <= 0:
            PWM_upper.ChangeDutyCycle(0)
            print(0)

    def lower_motor():
        dc_lower = M * x_lower + C
        if 100 >= dc_lower > 0:
            PWM_lower.ChangeDutyCycle(dc_lower)
            print(dc_lower)
        if dc_lower >= 100 or dc_lower <= 0:
            PWM_lower.ChangeDutyCycle(0)
            print(0)

    def right_motor():
        dc_right = M * x_right + C
        if 100 >= dc_right > 0:
            PWM_right.ChangeDutyCycle(dc_right)
            print(dc_right)
        if dc_right >= 100 or dc_right <= 0:
            PWM_right.ChangeDutyCycle(0)
            print(0)

    array = [x_left, x_upper, x_lower, x_right]
    idx = np.argmin(array)

    def motor_on():
        if idx == 0:
            left_motor()
            PWM_upper.ChangeDutyCycle(0)
            PWM_lower.ChangeDutyCycle(0)
            PWM_right.ChangeDutyCycle(0)

        if idx == 1:
            upper_motor()
            PWM_left.ChangeDutyCycle(0)
            PWM_lower.ChangeDutyCycle(0)
            PWM_right.ChangeDutyCycle(0)

        if idx == 2:
            lower_motor()
            PWM_left.ChangeDutyCycle(0)
            PWM_upper.ChangeDutyCycle(0)
            PWM_right.ChangeDutyCycle(0)

        if idx == 3:
            right_motor()
            PWM_left.ChangeDutyCycle(0)
            PWM_upper.ChangeDutyCycle(0)
            PWM_lower.ChangeDutyCycle(0)

    # Warning Function
    def warning_motor():
        for x in range(0, 3):
            print("Battery Low!!")
            PWM_left.ChangeDutyCycle(50)
            PWM_upper.ChangeDutyCycle(50)
            PWM_lower.ChangeDutyCycle(50)
            PWM_right.ChangeDutyCycle(50)
            time.sleep(1)
            PWM_left.ChangeDutyCycle(0)
            PWM_upper.ChangeDutyCycle(0)
            PWM_lower.ChangeDutyCycle(0)
            PWM_right.ChangeDutyCycle(0)
            time.sleep(1)

    # Print Out Function
    def print_function():
        print(x_left)
        print(x_upper)
        print(x_lower)
        print(x_right)
        print("------------")

    # Battery Moniter Statment
    if (GPIO.input(logic_A) == 0) and (GPIO.input(logic_B) == 0):
        motor_on()
        print_function()

    X = 0

    if (GPIO.input(logic_A) == 1) and (GPIO.input(logic_B) == 0):
        for x in range(0, 450):
            Y = 1
            X = Y + X
            if X == 1:
                warning_motor()
            if X> 1 and X <= 450:
                motor_on()
                print_function()

    if (GPIO.input(logic_A) == 1) and (GPIO.input(logic_B) == 1):
        warning_motor()
        break

GPIO.cleanup()
depth_stream.stop()
openni2.unload()
