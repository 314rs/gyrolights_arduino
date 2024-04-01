
# GyroLights

<img src="./docs/media/jumping led color.svg" align="right" align="left" width="100" height="100" />
A pair of pneumatic stilts, where both stilts are equipped with an ESP32, gyroscope and APA102 LED Strips. The LED effects can react to the movement using a accelerometer/gyroscope. Different modes can be set using a rotary switch on the master unit. The currently active mode is communicated to the slave unit via Bluetooth Low Energy.


## Installation

To install and run this project, follow these steps:

1. Clone this repository to your local machine.
1. Open the project in VSCode with the PlatformIO plugin installed
1. Edit the configuration files at `include/effectsConfig.h` and `include/projectConfig.h`
    - a pair of stilts must have the same `BLE_MASTER_NAME`
    - a pair of stilts can actually be exactly one master and any number of slaves
1. To select the master/slave sketch, select the environment accordingly. (eg. VSCode Statusbar in the bottom where its says `env:master (gyrolights_arduino)`)

## Documentation

Doxygen documentation can be found [here](https://314rs.github.io/gyrolights_arduino/).

## Todo

- [x] event driven
- [ ] e1.31 sACN
- [ ] ota update
- [ ] move bluetooth stack to nimble (maybe better with esp-idf buildsystem)
- [ ] refactor leds** to leds*
