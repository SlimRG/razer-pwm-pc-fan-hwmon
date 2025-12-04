# razer-pwm-pc-fan-hwmon
This repository provides an out-of-tree Linux kernel driver(Debian/Ubuntu) for the Razer PWM PC Fan Controller (USB HID, VID 0x1532 / PID 0x0f3c*. It exposes the controller via the standard `hwmon` sysfs interface, so you can control and monitor fans using lm-sensors / sensors / pwmconfig / fancontrol and other standard tools.
