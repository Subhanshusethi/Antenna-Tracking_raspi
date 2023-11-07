import RPi.GPIO as GPIO
from time import sleep
import math
from dronekit import connect

# Connecting the Vehicle
vehicle = connect('udpin:192.168.1.32:14552', baud=115200)

DIR1 = 10  # Direction pin for motor 1
STEP1 = 8  # Step pin for motor 1
DIR2 = 18  # Direction pin for motor 2
STEP2 = 16  # Step pin for motor 2
CW = 1  # Clockwise
CCW = 0  # Counterclockwise

# Setting up pins and output
GPIO.setmode(GPIO.BOARD)
GPIO.setup(DIR1, GPIO.OUT)
GPIO.setup(STEP1, GPIO.OUT)
GPIO.setup(DIR2, GPIO.OUT)
GPIO.setup(STEP2, GPIO.OUT)

current_step_motor1 = 0  # Variable to store current step for motor 1
current_step_motor2 = 0  # Variable to store current step for motor 2

steps_per_revolution = 200

def motor_control(motor_num, direction, steps, speed_delay, cycles, initial_step):
    count = 0
    current_step = initial_step
    steps = int(steps)
    if motor_num == 1:
        GPIO.output(DIR1, direction)
        while count < int(cycles):
            count += 1
            for _ in range(steps):
                GPIO.output(STEP1, GPIO.HIGH)
                sleep(speed_delay)
                GPIO.output(STEP1, GPIO.LOW)
                sleep(speed_delay)
                current_step += 1
    elif motor_num == 2:
        GPIO.output(DIR2, direction)
        while count < int(cycles):
            count += 1
            for _ in range(steps):
                GPIO.output(STEP2, GPIO.HIGH)
                sleep(speed_delay)
                GPIO.output(STEP2, GPIO.LOW)
                sleep(speed_delay)
                current_step += 1

    return current_step

def heading(lat1, long1, h1, lat2, long2, h2):
    # Calculation of heading, elevation angle, and steps
    lat1_rad = math.radians(lat1)
    long1_rad = math.radians(long1)
    lat2_rad = math.radians(lat2)
    long2_rad = math.radians(long2)

    dLon = (long2_rad - long1_rad)
    y = math.sin(dLon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dLon)
    brng = math.atan2(y, x)
    brng = math.degrees(brng)
    brng = (brng + 360) % 360
    brng = 360 - brng
    stepsh = (brng / 360) * steps_per_revolution

    R = 6371  # radius of the Earth in km

    d = math.acos(math.sin(lat1_rad) * math.sin(lat2_rad) +
                  math.cos(lat1_rad) * math.cos(lat2_rad) * math.cos(long2_rad - long1_rad)) * R

    if h1 != h2:
        if h1 > h2:
            z = h1 - h2
        elif h1 < h2:
            z = h2 - h1
        angle = math.atan2(z, d) * (180 / math.pi)
        print("Elevation Angle =", angle)
    else:
        angle = 0
        print(angle)
    steps = (angle / 360) * steps_per_revolution
    print("Steps taken =", steps)
    return brng, stepsh, angle, steps

# Variables to store previous data
prev_lat = 0
prev_long = 0
prev_alt = 0

# Update initial step values for each motor based on previous position
initial_step_motor1 = current_step_motor1
initial_step_motor2 = current_step_motor2

try:
    while True:
        a = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon,
             vehicle.location.global_relative_frame.alt]
        print(a)

        # Assigning previous data
        lat1 = prev_lat
        long1 = prev_long
        h1 = prev_alt

        lat2, long2, h2 = a  # Assigning new data

        threshold = 0.0001  # Adjust the threshold value as per your requirement

        if abs(lat1 - lat2) > threshold or abs(long1 - long2) > threshold or abs(h1 - h2) > threshold:
            brng, stepsh, angle, steps = heading(lat1, long1, h1, lat2, long2, h2)
            current_step_motor1 = motor_control(1, CW if brng > 180 else CCW, stepsh, 0.002, 1, initial_step_motor1)
            current_step_motor2 = motor_control(2, CW if angle > 0 else CCW, steps, 0.002, 1, initial_step_motor2)
            print("Pitch Angle =", angle)  # Print pitch angle
            print("Yaw Angle =", brng)  # Print yaw angle
        else:
            print("No movement required.")

        print('Task Completed')
        print("Waiting for next update...")
        sleep(0.15)

        # Updating previous data
        prev_lat = lat2
        prev_long = long2
        prev_alt = h2

except KeyboardInterrupt:
    print("Exiting program...")
finally:
    # Cleanup GPIO pins
    GPIO.cleanup()
