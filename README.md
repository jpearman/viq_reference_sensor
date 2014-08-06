viq_reference_sensor
====================

Example code to demonstrate a VEX IQ user sensor.

This code is designed to run on the MSP430 launchpad evaluation card.  It 
creates a sensor compatible with the VEX IQ brain with the following
functionality.

PWM control of the led connected to P1.0 (IO Port 1, Pin 0)
Monitoring of the switch connected to P1.3
Monitoring of the internal temperature sensor (ADC10, Channel 10)

The code can be compiled for either of the two MSP430 devices supplied with
the launchpad, the MSP430G2553 or the MSP430G2452.

Connection to the VEX IQ brain uses the i2c interface, this is typically
connected to pins P1.6 and P1.7 on an MSP430 device.

           MSP430G2553/G2452
          +-----------------+
       /|\|              XIN|-
        | |                 |
        --|RST          XOUT|-
          |                 |
          |P2.0         P1.0|-->LED
          |P2.1         P1.1|                  6P6C offset latch connector
          |P2.2         P1.2|                 +---------------------------+
          |P2.3         P1.3|<--Switch        | 1 DIO (not used)          |
          |P2.4         P1.4|                 | 5 7.2V (do not connect)   |
          |P2.5         P1.5|<----------------| 2 I2C_ENABLE              |
          |             P1.6|<----------------| 6 I2C_CLOCK               |
          |             P1.7|<--------------->| 3 I2C_DATA                |
          |                 |              -- | 4 GND                     |
          +-----------------+              |  +---------------------------+
                                          \|/

