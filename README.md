Vedder's open source bldc controller firmware with modifications mostly related to CAN bus communication.
## Compile options

Enable limit switch support. Connect limit switch to TX and RX pins on the P3 header on the Vesc.
Pull pin low to trigger limit switch, no need to pull high, Vesc has internal pullup.
make LIMIT_SWITCH=1

For Vesc with a 1k board thermistor instead of a 10k resistor
make ONE_K_THERM=1
