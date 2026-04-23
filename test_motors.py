import pico_4wd as car
import time

print("Testing Left Front Motor...")
car.set_motor_power(50, 0, 0, 0)
time.sleep(1)

print("Testing Right Front Motor...")
car.set_motor_power(0, 50, 0, 0)
time.sleep(1)

print("Testing Left Rear Motor...")
car.set_motor_power(0, 0, 50, 0)
time.sleep(1)

print("Testing Right Rear Motor...")
car.set_motor_power(0, 0, 0, 50)
time.sleep(1)

print("Stopping all motors...")
car.stop()
