# VTXFirmware
An arduino sketch to control a RTC6705 analog video transmitter

I calculated the register values for the corresponding frequencies, but the excel sheet to calculate them is in the repository. The sketch can't do much, but it is a start for everyone seeking a C fundation for their project.

I use the "standard" boscam module that is SPI ready
* PB3 - MOSI
* PB5 - SCK
* PC1 - CS

On PC0 is a button to controll the channel selection

Also there are 3 mosfets connected to the PWM pins of the atmega to controll LED stripes according to the channel selection
