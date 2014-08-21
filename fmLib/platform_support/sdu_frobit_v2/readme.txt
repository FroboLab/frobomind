This readme file contains information about the Frobit V2 interface component.





VOLTAGE MEASUREMENT
-------------------
The launch parameter "supply_voltage_scale_factor" defines the voltage divider
used on the microcontroller board. The default value corresponds to the default
RoboCard controller using resistor values of 1800/700 ohm for the voltage
divider. It is calculated using the formula:

5*(1800 + 700)/(700*1023) = 0.01746

The FroboMind Controller board uses 22000/3300 ohm for the voltage divider. In
order to support this board, the "supply_voltage_scale_factor" must therefore 
be set to the value:

5.0*(22000 + 3300)/(3300*1023) = 0.3747

