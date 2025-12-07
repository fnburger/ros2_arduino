/**
 * Smile Detector Arduino Receiver – Blinking Edition
 * 
 * Expected serial input (1 byte from ROS 2 node):
 *   0 → No face detected    → Blinking RED
 *   1 → Face detected       → Blinking GREEN  
 *   2 → Smile detected      → Blinking BLUE
 * 
 * Features:
 *   • Non-blocking blinking (250 ms on / 250 ms off → 500 ms cycle)
 *   • Instant color change on new command
 *   • Full brightness control (easy to dim if needed)
 *   • Serial feedback for debugging
 *   • Common-cathode RGB LED on PWM pins 9, 10, 11
 * 
 * Wiring (common cathode):
 *   Pin 9  → Red leg   (via 220–330 Ω resistor)
 *   Pin 10 → Green leg (via resistor)
 *   Pin 11 → Blue leg  (via resistor)
 *   Common cathode → GND
 */

const int redPin   = 9;    // PWM capable
const int greenPin = 10;   // PWM capable  
const int bluePin  = 11;   // PWM capable

// Current state
int currentColor = -1;     // -1 = off, 0=Red, 1=Green, 2=Blue
bool ledOn = false;

unsigned long previousMillis = 0;
const unsigned long BLINK_INTERVAL = 250;  // ms (250 on + 250 off = 2 Hz blink)

void setup() {
  Serial.begin(9600);
  
  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  // Ensure LED starts OFF
  allOff();

  Serial.println(F("Smile Detector Arduino Ready!"));
  Serial.println(F("Send: 0=No Face (Red blink) | 1=Face (Green blink) | 2=Smile (Blue blink)"));
}

void loop() {
  // === 1. Receive new command from ROS 2 ===
  if (Serial.available() > 0) {
    int received = Serial.read();

    if (received >= 0 && received <= 2) {
      if (received != currentColor) {
        currentColor = received;
        ledOn = false;                    // Reset blink phase on change
        previousMillis = millis();        // Synchronize blink timing
        Serial.print(F("→ Mode changed to: "));
        Serial.println(currentColor);
      }
    } else {
      Serial.println(F("Invalid code received (must be 0-2)"));
    }
  }

  // === 2. Handle blinking (non-blocking) ===
  if (currentColor != -1) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= BLINK_INTERVAL) {
      previousMillis = currentMillis;
      ledOn = !ledOn;      // Toggle every 250 ms

      if (ledOn) {
        setColor(currentColor);
      } else {
        allOff();
      }
    }
  }
  // If currentColor == -1 → stays off permanently
}

// Helper: Turn off all channels
void allOff() {
  analogWrite(redPin,   0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin,  0);
}

// Helper: Light only the selected color (0-255 brightness)
void setColor(int color) {
  allOff();  // Safety first
  const int brightness = 255;  // Change to 100-150 for softer blink

  switch (color) {
    case 0: analogWrite(redPin,   brightness); break;  // Red
    case 1: analogWrite(greenPin, brightness); break;  // Green
    case 2: analogWrite(bluePin,  brightness); break;  // Blue
  }
}