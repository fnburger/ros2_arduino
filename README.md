# ROS2 real-time smile detector using Arduino
Face & Smile Detection with Real-Time LED Feedback. This project uses a laptop webcam to detect human faces and smiles in real-time using OpenCV Haar cascades, running under ROS 2. Detection results are sent as simple integer codes over a ROS 2 topic chain and finally transmitted via serial to an Arduino, which lights up RGB LEDs accordingly:

| Detection State            | Code | LED Color |
|----------------------------|------|-----------|
| No face detected           | 0    | Red       |
| Face detected (no smile)   | 1    | Green     |
| Smile detected             | 2    | Blue      |


## Hardware Requirements

- Laptop/PC with built-in or USB webcam
- Arduino Uno / Nano / Mega (any board with hardware Serial/USB)
- Common-cathode RGB LED (common-anode also works with minor code change)
- 3× 220–330 Ω resistors
- Jumper wires & breadboard


## Example Wiring (Common-Cathode RGB LED)

| Arduino Pin | Component                  |
|-------------|----------------------------|
| 9           | Red leg (via resistor)     |
| 10          | Green leg (via resistor)   |
| 11          | Blue leg (via resistor)    |
| GND         | Common cathode             |