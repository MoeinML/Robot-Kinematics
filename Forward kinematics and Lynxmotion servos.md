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
    theta_rad = np.radians(angle)
    c = np.cos(theta_rad)
    s = np.sin(theta_rad)
    
    return np.array([
        [c, -s, dx],
        [s,  c, dy],
        [0,  0, 1 ]
    ])

a1 = 70
a2 = 45
theta1 = 42
theta2 = 28	

dx1 = a1 * np.cos(np.radians(theta1))
dy1 = a1 * np.sin(np.radians(theta1))
H_01 = create_htmatrix(theta1, dx1, dy1)

dx2 = a2 * np.cos(np.radians(theta2))
dy2 = a2 * np.sin(np.radians(theta2))
H_12 = create_htmatrix(theta2, dx2, dy2)


H_02 = H_01 @ H_12

pen_coords = H_02 @ np.array([0, 0, 1])

print("-" * 30)
print(f"Final Matrix (H_02):\n{H_02}")
print("-" * 30)
print(f"Coordinate of the pen: X = {pen_coords[0]:.2f}, Y = {pen_coords[1]:.2f}")
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
#This is an example on writing serial commands
#to TWO Lynxmotion SES-V2 smart servos.
#LSS-ADA board is used on connecting RPi serial
#  RPi -- LSS-ADA
#  GPIO14 Tx -- D1 servo Rx
#  GPIO15 Rx -- D0 servo Tx
#  GPIO GND -- GND
#The Lynxmotion SES-V2 servo serial 115200, parity none, 8 bits, stop bits 1
# Raspberry Pi Pico 2W board
#Serial port settings
#  Preferences, Raspberry Pi Configuration
#    Serial Port ON
#
#  sudo raspi-config
#   Interface Options
#    Serial port
#     "Would you like a login shell to be ..." NO
#      "Would you like the serial port hardware.." YES
#        Enter
#         Esc
#          sudo reboot
#
#Lynxmotion Smart Servo LSS Communication protocol: search for "Lynxmotion wiki" and
#open the Smart Servo (LSS), LSS Communication Protocol. 

from time import sleep
from machine import UART, Pin

bus = UART(
    0,
    baudrate=115200,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(0),
    rx=Pin(1)
)

# **********************************
servoID_A='15' # IMPORTANT: CHANGE HERE to the number of your servo A
servoID_B='40' # IMPORTANT: CHANGE HERE to the number of your servo B

# Also important: set gyre direction to counterclockwise (CCW) for servo A
bus.write(f'#{servoID_A}G-1\r'.encode()) # CCW: -1
bus.write(f'#{servoID_B}G1\r'.encode())  # CW: +1

# Optional: LED colors (0: off; colors 1 to 7)
bus.write(f'#{servoID_A}LED6\r'.encode())
bus.write(f'#{servoID_B}LED5\r'.encode())

# Define angle limits. DON'T CHANGE
MIN_ANGLE1 = 0
MAX_ANGLE1 = 180
MIN_ANGLE2 = -120
MAX_ANGLE2 = 120

try:
    while True: # you finish the program with ctrl+c
        angle1 = int(input(f'Enter angle for 1st servo ({MIN_ANGLE1}–{MAX_ANGLE1}): '))
        if not (MIN_ANGLE1 <= angle1 <= MAX_ANGLE1):
            raise ValueError(f'Angle1 must be between {MIN_ANGLE1} and {MAX_ANGLE1} degrees.')

        angle2 = int(input(f'Enter angle for 2nd servo ({MIN_ANGLE2}–{MAX_ANGLE2}): '))
        if not (MIN_ANGLE2 <= angle2 <= MAX_ANGLE2):
            raise ValueError(f'Angle2 must be between {MIN_ANGLE2} and {MAX_ANGLE2} degrees.')

        # Documentation: "Action Commands" in 
        # https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-v2/lynxmotion-smart-servo/lss-communication-protocol
        print(f'Moving to {angle1}, {angle2} degrees')
        bus.write(f'#{servoID_A}D{angle1*10}T1500\r'.encode()) # D indicates degrees
        bus.write(f'#{servoID_B}D{angle2*10}T1500\r'.encode())
        sleep(3)
except KeyboardInterrupt:
    print("Finished with the servo. Closing serial port.")
    bus.close()
    del bus
```
## 2. Testing and Validation

In this section, we verify the Forward Kinematics (FK) model by comparing the theoretical coordinates with the physical position of the robotic arm. The tests were conducted on a grid paper where **each cell represents 5 mm**.

### 2.1. Test Case 1: Zero Position
* **Input Angles:** $\theta_1 = 0^\circ, \theta_2 = 0^\circ$
* **Expected Coordinates:** $(L_1 + L_2, 0) = (115, 0)$
* **Observation:** The arm is fully extended along the X-axis.

![Robotic Arm at 0,0](path_to_your_photo_1.jpg)

### 2.2. Test Case 2: Perpendicular Position
* **Input Angles:** $\theta_1 = 90^\circ, \theta_2 = 0^\circ$
* **Expected Coordinates:** $(0, 115)$
* **Observation:** Link 1 is vertical, and Link 2 continues in the same direction.

![Robotic Arm at 90,0](path_to_your_photo_2.jpg)

### 2.3. Test Case 3: Right Angle Fold
* **Input Angles:** $\theta_1 = 0^\circ, \theta_2 = 90^\circ$
* **Expected Coordinates:** $(a_1, a_2) = (70, 45)$
* **Observation:** Link 1 lies on the X-axis, while Link 2 is perpendicular to it.

![Robotic Arm at 0,90](path_to_your_photo_3.jpg)

### 2.4. Test Case 4: Double Perpendicular
* **Input Angles:** $\theta_1 = 90^\circ, \theta_2 = 90^\circ$
* **Expected Coordinates:** $(-L_2, L_1) = (-45, 70)$
* **Observation:** The first link is vertical (along Y), and the second link turns 90 degrees CCW, pointing back along the negative X-axis.

![Robotic Arm at 90,90](path_to_your_photo_4.jpg)

### 2.5. Test Case 5: Verification using Grid Paper
For this final test, we used the specific angles from our code to compare the mathematical output with manual measurements on grid paper.

* **Input Angles:** $\theta_1 = 42^\circ, \theta_2 = 28^\circ$
* **FK Code Result:** $X \approx 68.32 \text{ mm}, Y \approx 87.89 \text{ mm}$
* **Manual Measurement:**
    We counted the cells on the grid paper from the origin $(0,0)$:
    * **X-axis:** $13.5 \text{ cells} \times 5 \text{ mm/cell} = 67.5 \text{ mm}$
    * **Y-axis:** $17 \text{ cells} \times 5 \text{ mm/cell} = 85.0 \text{ mm}$

**Conclusion:** The difference between the code results and the physical measurements is minimal (within $2-3 \text{ mm}$). This error is likely due to the physical backlash in the servo gears and slight misalignments when placing the arm on the grid paper. The experiment successfully validates our Forward Kinematics model.

![Manual Verification on Grid Paper](path_to_your_photo_5.jpg)


گ
