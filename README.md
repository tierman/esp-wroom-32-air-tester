# esp-wroom-32-air-tester
Small air tester based on esp-wroom-32 with sensor: CJMCU-8118, PMSA003, and display OLED 1306.

## External libraries:
* https://github.com/adafruit/Adafruit_CCS811
* https://github.com/maarten-pennings/CCS811
* https://github.com/adafruit/Adafruit-GFX-Library.git
* https://github.com/adafruit/Adafruit_SSD1306.git
* https://github.com/closedcube/ClosedCube_HDC1080_Arduino.git
* https://github.com/avaldebe/PMserial.git
* https://github.com/plerup/espsoftwareserial.git

## Electric configuration
* TP-4056 charger
* One battery 18650 (3,7V)
* HW-668 DC-DC Step up (5V for PMSA003 and esp-wroom-32)
* HW-613 DC-DC Step down (3.3V for other components)
* HW-221 level shifter (5v <-> 3.3V) (PMSA003 logic work only with 3.3V)
* Some simple voltage divider to measure battery voltage level, or direct measurement (without voltage divider).

## Error codes (onboard led blinking)
* 5 times - OLED display initialization failure
* 10 times - Gas sensor initialization failure