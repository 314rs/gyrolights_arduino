# GyroLight

A pair of pneumatic stilts, where both stilts are equipped with an ESP32, gyroscope and APA102 LED Strips. The LED effects can react to the movement using a accelerometer/gyroscope. Different modes can be set using a rotary switch on the master unit. The currently active mode is communicated to the slave unit via Bluetooth Low Energy.



## Uploading 
Es gibt ein Master und ein Slave programm. 
Ein Paar (bzw. eine Gruppe) von Stelzen muss den selben `BLE_MASTER_NAME` haben. Dieser kann in `projectConfig.h` gesetzt werden. In jeder Gruppe muss genau ein ESP32 als Master und beliebig viele Slaves sein. 



## Todo
- [ ] event driven
- [ ] e1.31 sACN
- [ ] ota update