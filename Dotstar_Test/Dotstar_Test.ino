// Simple test for Adafruit Dot Star RGB LED on SAMD21_USB by 10103.
// This is a basic diagnostic tool, NOT a graphics demo.

#include <Adafruit_DotStar.h> // APA102 library
#include <SPI.h>              // SPI comms for APA102

// Define input button pin
#define INPUTBTN  11          // Input button is on D11 - OR IS IT?!
// Define the LED pin
#define LED_PIN   13
// Define DOTSTAR DATA and CLOCK pins for the SAMD21_USB board
#define CLOCKPIN  40
#define DATAPIN   41
// There is only one pixel on the board
#define NUMPIXELS 1
// The maximum number of states
#define NUMSTATES 4

Adafruit_DotStar strip(1, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, so change this if you have an earlier strip.

int state = 0;               // state variable - 0 = APA102 rainbow cycle
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;  // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {

  // set IO pins
  pinMode(INPUTBTN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  // turn LED off initially
  digitalWrite(LED_PIN, LOW);
  // set up the APA102
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
  
}

void loop() {

  int reading = digitalRead(INPUTBTN);

  if (reading != lastButtonState) {
    // reset debounce timer
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      // only change state if the new button state is HIGH (released)
      if (buttonState == HIGH) {
        state += 1;
        if (state == NUMSTATES) {
          state = 0;
        }
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
  // GRB 
  if (state == 0) {        // GREEN
    //rainbow(30);
    digitalWrite(LED_PIN, HIGH);
    strip.setPixelColor(0, 100, 0, 0);
    strip.show();
  } else if (state == 1) { // AMBER
    digitalWrite(LED_PIN, LOW);
    strip.setPixelColor(0, 32, 100, 0);
    strip.show();
  } else if (state == 2) { // RED
    digitalWrite(LED_PIN, LOW);
    strip.setPixelColor(0, 0, 100, 0);
    strip.show();
  } else if (state == 3) { // AMBER
    digitalWrite(LED_PIN, LOW);
    strip.setPixelColor(0, 32, 100, 0);
    strip.show();
  }

}
