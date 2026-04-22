# Forward Kinematics and Lynxmotion Servos

## Group Name: **ServoFK 5**  
**Group Members:** 
* Moein Lahashinejad
* Navid Behazin
---
## 1.A. Forward Kinematics (FK) Calculation

### Description
The following Python script calculates the position of the robot's end-effector (the pen) in a 2D plane. By providing the lengths of the two links ($a_1, a_2$) and the joint angles ($\theta_1, \theta_2$), the program constructs Homogeneous Transformation Matrices (HTM). It then performs matrix multiplication to determine the final coordinates $(X, Y)$ relative to the base frame.

### Python Code Implementation
```python
import numpy as np

def create_htmatrix(angle, dx, dy):
    """
    Creates a 3x3 Homogeneous Transformation Matrix for 2D space.
    - angle: Rotation in degrees
    - dx, dy: Translation offsets
    """
    theta_rad = np.radians(angle)
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    
    return np.array([
        [c, -s, dx],
        [s,  c, dy],
        [0,  0, 1 ]
    ])

# Link lengths (units from robot specifications)
a1 = 70
a2 = 45

# Joint angles in degrees
theta1 = 42
theta2 = 28	

# Step 1: Calculate H_01 (Base to Link 1)
dx1 = a1 * np.cos(np.radians(theta1))
dy1 = a1 * np.sin(np.radians(theta1))
H_01 = create_htmatrix(theta1, dx1, dy1)

# Step 2: Calculate H_12 (Link 1 to Link 2)
dx2 = a2 * np.cos(np.radians(theta2))
dy2 = a2 * np.sin(np.radians(theta2))
H_12 = create_htmatrix(theta2, dx2, dy2)

# Step 3: Global Transformation Matrix (Matrix Multiplication)
H_02 = H_01 @ H_12

# Step 4: Extract final coordinates of the pen
pen_coords = H_02 @ np.array([0, 0, 1])
```

## 1.B. Running the Servos (Hardware Implementation)

### Description
This script is designed to control the physical movement of the robotic arm. It must be executed using the **MicroPython interpreter** (e.g., on a Raspberry Pi Pico or RPi Zero in MicroPython mode). 

The program operates in an interactive loop:
1. It establishes a serial connection via **UART** with a baud rate of 115200.
2. It initializes the servos by setting their gyre direction and LED colors.
3. In each iteration, it prompts the user to input two angles: one for the first link (Servo A) and one for the second link (Servo B).
4. The input degrees are converted to **tenths of degrees** (as required by the LSS protocol) and sent to the motors.

### MicroPython Code Implementation
```python
# This script controls two Lynxmotion SES-V2 smart servos via UART.
# Target Environment: MicroPython (Raspberry Pi Pico/Zero 2W)

from time import sleep
from machine import UART, Pin

# Initialize UART communication
bus = UART(
    0,
    baudrate=115200,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(0),
    rx=Pin(1)
)

# Servo Configuration
servoID_A = '15' # ID for Link 1
servoID_B = '40' # ID for Link 2

# Set initial directions and LED status
bus.write(f'#{servoID_A}G-1\r'.encode()) # Set CCW for Servo A
bus.write(f'#{servoID_B}G1\r'.encode())  # Set CW for Servo B
bus.write(f'#{servoID_A}LED6\r'.encode())
bus.write(f'#{servoID_B}LED5\r'.encode())

# Define safety angle limits
MIN_ANGLE1, MAX_ANGLE1 = 0, 180
MIN_ANGLE2, MAX_ANGLE2 = -120, 120

try:
    while True:
        # Request user input for real-time control
        angle1 = int(input(f'Enter angle for 1st servo ({MIN_ANGLE1} to {MAX_ANGLE1}): '))
        if not (MIN_ANGLE1 <= angle1 <= MAX_ANGLE1):
            raise ValueError("Angle1 out of range.")

        angle2 = int(input(f'Enter angle for 2nd servo ({MIN_ANGLE2} to {MAX_ANGLE2}): '))
        if not (MIN_ANGLE2 <= angle2 <= MAX_ANGLE2):
            raise ValueError("Angle2 out of range.")

        print(f'Moving servos to {angle1} and {angle2} degrees...')
        
        # Command format: #<ID>D<tenths of degrees>T<time in ms>
        bus.write(f'#{servoID_A}D{angle1*10}T1500\r'.encode())
        bus.write(f'#{servoID_B}D{angle2*10}T1500\r'.encode())
        
        sleep(3)
        
except KeyboardInterrupt:
    print("Finished with the servo. Closing connection.")
    # Connection cleanup is handled by the interpreter on exit

# Output Results
print("-" * 30)
```

print(f"Final Matrix (H_02):\n{H_02}")
print("-" * 30)
print(f"Coordinate of the pen: X = {pen_coords[0]:.2f}, Y = {pen_coords[1]:.2f}")
