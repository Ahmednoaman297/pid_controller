# Dual Motor Control with PID Regulation

This Arduino project controls two motors with PID regulation, maintaining a target position and velocity. The code utilizes encoders for feedback, PWM for speed control, and direction pins for controlling the rotation direction of the motors. This implementation allows precise control of each motor, ensuring they reach and maintain their desired positions and velocities.

## Hardware Configuration

### Motor Right
- **Encoder Pins:** 2 and 3
- **PWM Pin:** 5
- **Direction Pin:** 7

### Motor Left
- **Encoder Pins:** 21 and 20
- **PWM Pin:** 44
- **Direction Pin:** 45

## Constants and Variables

### Motor Right
- **PID Constants:** 
  - `proportional = 1`
  - `kpv = 1`
  - `kiv = 40`
- **State Variables:**
  - `motorPosition` - Position of the motor based on encoder readings
  - `velocity` - Current velocity of the motor
  - `controlSignal` - Control signal for the motor
  - `target_pos_cm` - Target position in centimeters
  - `desiredvelocity` - Desired velocity

### Motor Left
- **PID Constants:** 
  - `proportional_left = 1`
  - `kpv_left = 1`
  - `kiv_left = 49.5`
- **State Variables:**
  - `motorPosition_left` - Position of the motor based on encoder readings
  - `velocity_left` - Current velocity of the motor
  - `controlSignal_left` - Control signal for the motor
  - `target_pos_cm_left` - Target position in centimeters
  - `desiredvelocity_left` - Desired velocity

## Functions

### `setup()`
Initializes serial communication and pin modes. Attaches interrupts for encoder readings.

### `loop()`
Main control loop running PID calculations and applying control signals to both motors. It computes the current velocity and position, calculates the control signals using the PID algorithm, and updates the PWM and direction pins accordingly.

### `checkEncoder1()`, `checkEncoder2()`, `checkEncoder1_left()`, `checkEncoder2_left()`
Interrupt service routines for reading encoder pulses and updating motor positions. These functions are called whenever an encoder pulse is detected.

## PID Control Logic
1. **Velocity Calculation:** 
   - Computes the current velocity based on encoder readings.
2. **Control Signal Calculation:**
   - Calculates the control signal to reach the target position.
   - Limits the control signal to the desired velocity.
3. **Error Calculation:**
   - Calculates the velocity error and integrates it over time.
4. **PWM Value Calculation:**
   - Converts the control signal to a PWM value for motor control.
   - Caps the PWM value at 255 to prevent overflow.
5. **Motor Direction Control:**
   - Sets the motor direction based on the sign of the control signal.

## Usage

1. **Compile and Upload:**
   - Load the code onto your Arduino board.
2. **Run:**
   - Open the Serial Monitor to view real-time velocity and control signals.
3. **Adjust Parameters:**
   - Modify the PID constants and target positions/velocities as needed for your specific application.

## Notes

- Ensure the encoders are properly connected to the specified pins.
- Fine-tune the PID constants (`proportional`, `kpv`, `kiv`) for optimal performance based on your motors and application requirements.
- Monitor the serial output to verify the motor's behavior and make necessary adjustments.

This project provides a robust foundation for precise motor control applications, suitable for robotics, automation, and other electromechanical systems.
