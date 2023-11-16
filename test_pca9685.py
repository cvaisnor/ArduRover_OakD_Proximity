import time
from adafruit_servokit import ServoKit

print("Initializing ServoKit")
kit = ServoKit(channels=16)

print('Setting steering and throttle channels')
steering = kit.servo[0]
steering.angle = 90 # set steering to neutral position

# channel 1 needs to be set for PWM DC motor control
throttle = kit.continuous_servo[1]
throttle.throttle = 0 # set throttle to neutral position

throttle_values = [0.06, 0.07] # 0.06 = stop, 0.07 = forward # DO NOT GO ABOVE 0.07 TOO FAST LOL

def main():
    print('Starting main loop')
    # wait for the user to press c
    # ensures the ESC calibration is done before starting the loop
    if input("Press c to continue") == 'c':
        print("Starting loop")
        while True:
            # print("Setting steering to neutral position")
            # steering.angle = 90
            # print("Setting throttle to minimum speed")
            throttle.throttle = throttle_values[1]

if __name__ == "__main__":
    main()