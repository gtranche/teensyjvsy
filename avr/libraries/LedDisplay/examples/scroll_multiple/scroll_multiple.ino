/*
  HCMS Display
 Language: Arduino/Wiring

 Displays a string on three Avago HCMS-297x displays.
 Scrolls the string left and right.

 created 12 Jun. 2008
 modified 11 March 2010
 by Tom Igoe

 */
#include <LedDisplay.h>

// Define pins for the LED display.
// You can change these, just re-wire your board:
#define dataPin 6              // connects to the display's data in
#define registerSelect 7       // the display's register select pin
#define clockPin 8             // the display's clock pin
#define enable 9               // the display's chip enable pin
#define reset 10              // the display's reset pin

#define displayLength 24        // total number of characters in all three displays


// create am instance of the LED display:
LedDisplay myDisplay = LedDisplay(dataPin, registerSelect, clockPin,
                                  enable, reset, displayLength);

int brightness = 15;           // screen brightness
int myDirection = 1;           // direction of scrolling.  -1 = left, 1 = right.

void setup() {
  Serial.begin(9600);
  // initialize the display library:
  myDisplay.begin();
  myDisplay.clear();
  myDisplay.setString("Aardvarks mark the park after dark.");
  myDisplay.setBrightness(brightness);
  delay(100);
}

void loop() {

  // when the string scrolls off the display, reverse scroll direction.
  // On the right, it scrolls off at position 8.
  // on the left, it scrolls off when the cursor is less than -(the length of the string):
  if ((myDisplay.getCursor() > displayLength) ||
    (myDisplay.getCursor() <= -(myDisplay.stringLength()))) {
    myDirection = -myDirection;
    delay(1000);
  }

  // scroll:
  myDisplay.scroll(myDirection);
  delay(100);
}
