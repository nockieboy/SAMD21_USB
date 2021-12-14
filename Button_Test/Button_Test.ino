// Button_Test for SAMD21_USB by 10103.
// This is a basic diagnostic tool which will scan an array
// if IO pins specified in the result[] array, and when the
// value of any of those IO pins changes, it will flash the
// D13 LED the appropriate number of times to indicate the
// IO pin value.

#include <Adafruit_DotStar.h> // APA102 library
#include <SPI.h>              // SPI comms for APA102

// Define the LED pin
#define LED_PIN   13
// Define DOTSTAR DATA and CLOCK pins for the SAMD21_USB board
#define CLOCKPIN  40
#define DATAPIN   41
// There is only one pixel on the board
#define NUMPIXELS 1
// The maximum number of states
#define NUMSTATES 3

Adafruit_DotStar strip(1, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, so change this if you have an earlier strip.

int state = 0;               // state variable - 0 = APA102 rainbow cycle
int ledState = LOW;          // the current led value
int lastButtonState = HIGH;  // the previous reading from the input pin

// Define input button pins and result arrays
int inputPins[10] = {6,7,8,9,10,11,12,14,15,16};
int result[10] = {1,1,1,1,1,1,1,1,1,1};
int oldres[10] = {1,1,1,1,1,1,1,1,1,1};

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // set IO pins
  for (int i=0; i<10; i++) {
     pinMode(inputPins[i],INPUT);
  }
  pinMode(LED_PIN, OUTPUT);
  // set LED to initial value
  digitalWrite(LED_PIN, ledState);
  // set up the APA102
  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP

  for (int i=0; i<10; i++) {
    digitalWrite(LED_PIN, HIGH);
    oldres[i] = digitalRead(inputPins[i]);
    delay(100); // small delay for stability
    digitalWrite(LED_PIN, LOW);
  }
}

void loop() {

  for (int i=0; i<10; i++) {
    result[i] = digitalRead(inputPins[i]);
    if (result[i] != oldres[i]) {
      //Serial.println((String)"Pin "+i+" changed value!");
      FlashLED(inputPins[i]);
      oldres[i] = result[i];
    }
    delay(1); // small delay for stability
  }
  
}

void FlashLED(int times) {

  digitalWrite(LED_PIN, LOW);
  delay(1000);
  
  for (int i=0; i<times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
}
