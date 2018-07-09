# Sonar Device

This device consists of a sonar array of 5 HC-SR04 elements controlled
by an arduino and connected via USB-Serial to the host robot.

## Bill of materials:

* HC-SR04 (5)
* Arduino Nano (or equivalent)
* 4pin connector for SR04
* USB-Mini-B connector for host-to-nano
* 3d printed housing (see STL files).

## How to build it

Detailed instructions perhaps later.  In a nutshell, link the "Trigger"
and "Echo" pins to the Nano's data pins and provide +5 and GND to sensors.

Then program the Nano with the accompanying firmware.  Then connect it via USB
to your robot host and start the ROS driver.

Using a Prusa or equivalent 3d printer, print the housing and assemble the circuit
into the housing.
