Vedder's open source bldc controller firmware with modifications mostly related to CAN bus communication.
## Compile options

Enable limit switch support. Connect limit switch to TX and RX pins on the P3 header on the Vesc.
Pull pin low to trigger limit switch, no need to pull pin high, Vesc has internal pullup.

    make LIMIT_SWITCH=1

For Vesc with a 1k board thermistor instead of a 10k resistor

    make ONE_K_THERM=1

## Custom Control Loop
In `applications/app_example.c` there is a thread named `CUSTOM_CONTROL`. `custom_setpoint` is the variable set by `setCustom()`from https://github.com/DangerousElectrician/vescSocketCAN 

`applications/app_example.c` is also where most of the CAN bus control code exists. Some more code is in `comm_can.c` to handle receiving CAN bus data.
