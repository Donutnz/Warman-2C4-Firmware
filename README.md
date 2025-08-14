# Warman-2C4-Firmware

# Description

Firmware for SLAPBOT.

Warman 2025 Group 2C4 bot.

# Usage
- Wire stepper motor coils identically. Reversing is handled in software.
- In the task list, delays must be put at the beginning of the next task rather than the end of the current task to work properly. This is due to how things are executed.

# Dependencies
- PlatformIO
- SpeedyStepper
- ServoEasing
- 

# TODO
- Develop task list
- Finalise task list
- Create pathing
- Add tone module support
- Tune smooth steppers
- Tune servo positions
- Delay after start button (w beep on actual start)
- Slow down accel
- Slow down accel/speed on turning

# Maybe todo
- Second LED?
- Second switch?
- Queable servo positions?
    - Allows multiple servo moves during continous wheel movement.
- Sync firing of servo moves (specifically arms) to stepper posistion rather than task. Would allow multiple servo moves in one stepper move.
    - Use if/else if with stepper.getCurrentPositionInMillimeters() method.

# Done
- Neutral turn not working
- Build task list
- Test task list
- Add LED flashes for states
- Start up sequence
    - On
    - Test
    - Ready
    - Go!
- Smooth servos (servoEasing library)
    - Check if moves can be done async (yes, also done)
- Debug serial output?
- Switch reading

# BUGS
- R Stepper direction pin is pin 1, which is one of the arduino serial pins. This causes issues with programming and serial output.

# Notes
- Test in Wokwi