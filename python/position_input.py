import time
import math
import sys
import select  # Import select for non-blocking input
from pynput import keyboard  # For key handling
sys.path.append('../lib')
from unitree_actuator_sdk import *

# Initialize serial communication and motor command
serial = SerialPort('/dev/ttyUSB3')
cmd = MotorCmd()
data = MotorData()

# Initialize parameters
gearratio = queryGearRatio(MotorType.GO_M8010_6)
desired_position = 0.0  # Initial desired position in radians
kp_min = 0.1  # Minimum proportional gain
kp_max = 1.0  # Maximum proportional gain
kd = 0.01  # Derivative gain
position_step = 10 * math.pi / 180   # 10 degrees in radians
key_flags = {"increase": False, "decrease": False}  # Flags for key presses

# Function to prompt the user for a position
def prompt_for_position():
    global desired_position
    try:
        user_input = float(input("Enter desired position in radians (0 to 2*pi): "))
        if user_input < 0:
            user_input = 0
        elif user_input > 2 * math.pi:
            user_input = 2 * math.pi
        desired_position = user_input * gearratio  # Scale to motor units
        print(f"Setting desired position to: {math.degrees(desired_position / gearratio):.2f} degrees")
    except ValueError:
        print("Invalid input. Please enter a valid number.")

# Key press event handler
def on_press(key):
    global key_flags
    try:
        if key == keyboard.Key.right:  # Increase position
            key_flags["increase"] = True
        elif key == keyboard.Key.left:  # Decrease position
            key_flags["decrease"] = True
    except AttributeError:
        pass

# Key release event handler
def on_release(key):
    global key_flags
    if key == keyboard.Key.right:
        key_flags["increase"] = False
    elif key == keyboard.Key.left:
        key_flags["decrease"] = False

# Listener for keyboard events
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("Press Right Arrow to increase position by 10 degrees, Left Arrow to decrease position by 10 degrees.")
print("Press 'Ctrl+C' to exit or type a position in radians (0 to 2*pi) at the prompt.")

# try:
#     while True:
#         # Check for key flags and update desired position
#         if key_flags["increase"]:
#             desired_position += position_step
#             desired_position = min(desired_position, 2 * math.pi * gearratio)  # Clamp to max
#             print(f"Increasing desired position to: {math.degrees(desired_position / gearratio):.2f} degrees")
#             key_flags["increase"] = False
#         if key_flags["decrease"]:
#             desired_position -= position_step
#             desired_position = max(desired_position, 0)  # Clamp to min
#             print(f"Decreasing desired position to: {math.degrees(desired_position / gearratio):.2f} degrees")
#             key_flags["decrease"] = False

#         # Check for user input to set position
#         if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:  # Check if input is ready
#             prompt_for_position()

#         # Read current motor position
#         data.motorType = MotorType.GO_M8010_6
#         serial.sendRecv(cmd, data)
#         current_position = data.q  # Current motor position in radians

#         # Compute position error
#         position_error = abs(desired_position - current_position)

#         # Adjust kp dynamically based on the error
#         if position_error > 1.0:  # Large error
#             kp = kp_min
#         else:  # Small error
#             kp = kp_max * (1 - position_error) + kp_min * position_error

#         # Command motor to move to the desired position
#         cmd.motorType = MotorType.GO_M8010_6
#         cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
#         cmd.id = 0
#         cmd.q = desired_position
#         cmd.dq = 0.0
#         cmd.kp = kp
#         cmd.kd = kd
#         cmd.tau = 0

#         # Send command to the motor
#         serial.sendRecv(cmd, data)

#         # # Print debugging information
#         # print('\n')
#         # print(f"Current Position (q): {data.q / gearratio:.2f} degrees")
#         # print(f"Desired Position: {desired_position / gearratio:.2f} degrees")
#         # print(f"Position Error: {position_error:.2f} radians")
#         # print(f"Current Kp: {kp:.2f}")
#         # print(f"Current Velocity (dq): {data.dq / gearratio:.2f} degrees/s")
#         # print(f"Temperature: {data.temp:.2f} C")
#         # print(f"Error Status: {data.merror}")
#         # print('\n')

#         # Small delay to control loop rate
#         time.sleep(0.01)  # 10 ms delay

# except KeyboardInterrupt:
#     print("\nScript interrupted by user. Exiting gracefully...")
# finally:
#     # Safely stop the motor
#     cmd.q = data.q  # Hold the current position
#     cmd.dq = 0.0
#     cmd.tau = 0.0
#     serial.sendRecv(cmd, data)
#     print("Motor stopped.")


# Inside the main try block, before sending the command to the motor
try:
    while True:
        # Check for key flags and update desired position
        if key_flags["increase"]:
            desired_position += position_step
            desired_position = min(desired_position, 2 * math.pi * gearratio)  # Clamp to max
            print(f"Increasing desired position to: {math.degrees(desired_position / gearratio):.2f} degrees")
            key_flags["increase"] = False
        if key_flags["decrease"]:
            desired_position -= position_step
            desired_position = max(desired_position, 0)  # Clamp to min
            print(f"Decreasing desired position to: {math.degrees(desired_position / gearratio):.2f} degrees")
            key_flags["decrease"] = False

        # Check for user input to set position
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:  # Check if input is ready
            prompt_for_position()

        # Read current motor position
        data.motorType = MotorType.GO_M8010_6
        serial.sendRecv(cmd, data)
        current_position = data.q  # Current motor position in radians

        # Log current and desired positions
        print(f"Current Position: {math.degrees(current_position / gearratio):.2f} degrees, "
              f"Desired Position: {math.degrees(desired_position / gearratio):.2f} degrees")

        # Compute position error
        position_error = abs(desired_position - current_position)

        # Adjust kp dynamically based on the error
        if position_error > 1.0:  # Large error
            kp = kp_min
        else:  # Small error
            kp = kp_max * (1 - position_error) + kp_min * position_error

        # Command motor to move to the desired position
        cmd.motorType = MotorType.GO_M8010_6
        cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        cmd.id = 0
        cmd.q = desired_position
        cmd.dq = 0.0
        cmd.kp = kp
        cmd.kd = kd
        cmd.tau = 0

        # Send command to the motor
        serial.sendRecv(cmd, data)

        # Small delay to control loop rate
        time.sleep(0.01)  # 10 ms delay

except KeyboardInterrupt:
    print("\nScript interrupted by user. Exiting gracefully...")
finally:
    # Safely stop the motor
    cmd.q = data.q  # Hold the current position
    cmd.dq = 0.0
    cmd.tau = 0.0
    serial.sendRecv(cmd, data)
    print("Motor stopped.")
