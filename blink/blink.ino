// Blink for SAMD21_USB by 10103.
//
// Although this is just an LED 'blink' test, it also
// references the APA102 via the DotStar library to turn
// the APA102 off, so could be considered a test of the
// DotStar/APA102 as well - even though it's not the
// most colourful way to test that component!

#include <Adafruit_DotStar.h>
#include <SPI.h>

// Define DOTSTAR DATA and CLOCK pins for the SAMD21_USB board
#define DATAPIN    41
#define CLOCKPIN   40

// There is only one pixel on the board
#define NUMPIXELS 1

// We reference the APA102 LED on the board so that we can turn it off,
// otherwise it defaults to a distractingly bright pinky-purple colour.
Adafruit_DotStar strip(1, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

int onoff = LOW;

void setup() {
  
  pinMode(13, OUTPUT);
  strip.begin(); // Initialize pins for output
  strip.setBrightness(25);
  strip.show();  // Turn all LEDs off ASAP
  
}

void loop() {
  
  blinkLED(1000);
  
}

void blinkLED (int wait) {

  delay (wait);
  onoff = !onoff;
  digitalWrite(13, onoff);
  
}
