# GPIO-ADC Bridge for Crazyflie 2.x running Aerolink

Edited: <milan_capoor@brown.edu>, 2 Oct 2026

This folder contains the app layer application for the Crazyflie to send output on a GPIO and read ADC values, sending the data to the [cfclient](https://github.com/bitcraze/crazyflie-clients-python).

This proof-of-concept build exposes IO1-4 and writes "Lateral connected" (aerolink.lateral) if IO1 is connected to IO3 and "Vertical connected" (aerolink.vertical) if IO2 is connected to IO4. Sampling rate is 50 ms (20 Hz) with a 5 sample debouncing.

See App layer API guide and build instructions [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)
