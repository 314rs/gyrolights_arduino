# GyroLight

An ESP32 that controlls WS2812 LED strips on pneumatic stilts. The LED effects can react to the movement using a accelerometer/gyroscope. Different modes can be set using a rotary switch. Alternativeley, the ESP32 can receive wireless DMX data over E1.31. The ESP32 can be updated OTA. 


## Architecture Ideas

__Use the button library from esp-idf-lib for MVP!!!__

### Task1: user input
``` 
    - read user input (rf switch + rotary switch)
    - debounce user input
    - push to queue
```

### Task1.2: telnet input
same as above, but over telnet. decode delnet commands and push to eventqueue

### Task2: logic 
```
    - read queue
    - if nothing, return
    - if anything:
        - check against global state
        - if sth should happen, do so.
```

## Connection between stilts

Talk via ESP-NOW? drawback: esp now can operate exclusively on the same channel as wifi


## Todo
- [x] ruck? effekt
- [ ] zweiter ESP, Funkverbindung
- [ ] rotary Switch einlesen
- [ ] state management
- [ ] effekte in config
- [ ] ota update
