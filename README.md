## Main TeleOp for FTC Robot Code

This file defines the primary TeleOp mode for the robot, designed to control its various mechanisms, including a mecanum drive system, arm, slider, and two servos. The code integrates multiple components with smooth controls for competitive or practice use. This code is owned and written by the Programming Team of Tech Infinity #25005. The robot using this code is based on the Strafer Chassis from goBilda.

## Motors and Their Functions

### Drivetrain Motors

- **frontLeftMotor (fl_drive):**
  - Front left wheel of the mecanum drive system.
  - Responsible for forward/backward, strafing, and turning movements.

- **frontRightMotor (fr_drive):**
  - Front right wheel of the mecanum drive system.
  - Works in coordination with other drivetrain motors for smooth movement.

- **backLeftMotor (bl_drive):**
  - Back left wheel of the mecanum drive system.
  - Completes the four-wheel drive setup for omnidirectional control.

- **backRightMotor (br_drive):**
  - Back right wheel of the mecanum drive system.
  - Ensures precise turns and strafing with proper power distribution.

### Arm Motor

- **arm (arm):**
  - Controls the robotic arm’s movement.
  - Equipped with encoder feedback for precise positioning.
  - Uses a PID controller to hold the arm in position when the joystick is released.

### Slider Motor

- **slider (slider):**
  - Operates the linear slider mechanism.
  - Controlled by the left stick X-axis of Gamepad 2.

### Servos and Their Functions

- **270° Servo:**

  - **servo1 (wrist servo):**
    - Can move to two specific positions based on button inputs from Gamepad 2:
      - **Button A:** Folds the wrist.
      - **Button B:** Folds the servo to the specimen drop position.

- **Continuous Rotation Servo:**

  - **servo2 (intake continuous rotation servo):**
    - Controlled by the right stick Y-axis of Gamepad 2.
    - Provides continuous rotation for the active intake.

## Key Features

### Mecanum Drive Control

- Uses joystick inputs from Gamepad 1 to enable:
  - **Forward/Backward:** Controlled by the left joystick Y-axis.
  - **Strafing:** Controlled by the left joystick X-axis.
  - **Turning:** Controlled by the right joystick X-axis.
- Ensures smooth movement using proportional power distribution to all four wheels.

### Arm PID Control

- Uses a proportional-integral-derivative (PID) controller for precise arm positioning.
- Allows:
  - Direct joystick control when actively moved.
  - Position holding when the joystick is released, maintaining stability.

### Slider Control

- Operates using the left joystick X-axis of Gamepad 2 for smooth and intuitive linear motion.

### Servo Control

- Allows fine-grained control of servos for specific mechanisms or tasks.

### Telemetry

- Provides real-time data for debugging and performance monitoring, including:
  - Arm target and current positions.
  - Arm PID error.
  - Servo1 position.

This TeleOp program ensures optimal control of the robot's various mechanisms during operation, leveraging advanced features like PID control and mecanum drive calculations for competitive performance.
