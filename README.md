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
- Second LED?
- Second switch?
- Tune smooth steppers
- Tune servo positions
- Queable servo positions?
    - Allows multiple servo moves during continous wheel movement.

# Done
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


# Notes
- Test in Wokwi