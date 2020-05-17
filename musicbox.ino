#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>

// These are the pins used
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)

// Feather ESP8266
#if defined(ESP8266)
  #define VS1053_CS      16     // VS1053 chip select pin (output)
  #define VS1053_DCS     15     // VS1053 Data/command select pin (output)
  #define CARDCS          2     // Card chip select pin
  #define VS1053_DREQ     0     // VS1053 Data request, ideally an Interrupt pin

// Feather ESP32
#elif defined(ESP32)
  #define VS1053_CS      32     // VS1053 chip select pin (output)
  #define VS1053_DCS     33     // VS1053 Data/command select pin (output)
  #define CARDCS         14     // Card chip select pin
  #define VS1053_DREQ    15     // VS1053 Data request, ideally an Interrupt pin

// Feather Teensy3
#elif defined(TEENSYDUINO)
  #define VS1053_CS       3     // VS1053 chip select pin (output)
  #define VS1053_DCS     10     // VS1053 Data/command select pin (output)
  #define CARDCS          8     // Card chip select pin
  #define VS1053_DREQ     4     // VS1053 Data request, ideally an Interrupt pin

// WICED feather
#elif defined(ARDUINO_STM32_FEATHER)
  #define VS1053_CS       PC7     // VS1053 chip select pin (output)
  #define VS1053_DCS      PB4     // VS1053 Data/command select pin (output)
  #define CARDCS          PC5     // Card chip select pin
  #define VS1053_DREQ     PA15    // VS1053 Data request, ideally an Interrupt pin

#elif defined(ARDUINO_NRF52832_FEATHER )
  #define VS1053_CS       30     // VS1053 chip select pin (output)
  #define VS1053_DCS      11     // VS1053 Data/command select pin (output)
  #define CARDCS          27     // Card chip select pin
  #define VS1053_DREQ     31     // VS1053 Data request, ideally an Interrupt pin

// Feather M4, M0, 328, nRF52840 or 32u4
#else
  #define VS1053_CS       6     // VS1053 chip select pin (output)
  #define VS1053_DCS     10     // VS1053 Data/command select pin (output)
  #define CARDCS          5     // Card chip select pin
  // DREQ should be an Int pin *if possible* (not possible on 32u4)
  #define VS1053_DREQ     9     // VS1053 Data request, ideally an Interrupt pin

#endif

Adafruit_VS1053_FilePlayer musicPlayer = 
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);

const int buttonPin = 12;
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

bool serial = false;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; modify as needed

void setup() {
  pinMode(buttonPin, INPUT_PULLUP); // or change to INPUT if you are not lazy and do this in hardware
  if (serial) { Serial.begin(115200);
    while (!Serial) { delay(1); }
    delay(500);
    Serial.println("\n\nmusic box using VS1053 is here");
  }
  if (! musicPlayer.begin()) { // initialise the music player
    if(serial) {
      Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
    }
    while (1);
  }

  if (serial) {
    Serial.println(F("VS1053 found"));
  }
 
  if (!SD.begin(CARDCS)) {
    if (serial) {
      Serial.println(F("SD failed, or not present"));
    }
    while (1);  // don't do anything more
  }
  if (serial) {
    Serial.println("SD OK!");
  }
   
  musicPlayer.setVolume(30,30); // this was about as loud as I could go without distortion but YMMV
  
#if defined(__AVR_ATmega32U4__) 
  // Timer interrupts are not suggested, better to use DREQ interrupt!
  // but we don't have them on the 32u4 feather...
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT); // timer int
#else
  // If DREQ is on an interrupt pin we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
#endif
}

void loop() {
  // File is playing in the background
  if (musicPlayer.stopped()) {
    if(serial) {
      Serial.println("Not playing music");
    }
    int reading = digitalRead(buttonPin);
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {
        buttonState = reading;
        if (buttonState == LOW) { // checking for LOW rather than HIGH was more reliable for me
          if (serial) {
            Serial.println("btn LOW now");
          }
          musicPlayer.playFullFile("/track001.mp3");
        } else {
          if (serial) {
            Serial.println("btn HIGH now");
          }
        }
      }
    }
    lastButtonState = reading;
  } else {
    // if you don't have the 32u4 and therefore have DREQ interrupts you get to stop the music!
    // I do have the 32u4 tho so maybe this doesn't work
    if (serial) {
      Serial.println("Playing music");
    }
    int reading = digitalRead(buttonPin);
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {
        buttonState = reading;
        if (buttonState == LOW) {
          if (serial) {
            Serial.println("btn LOW now");
          }
          musicPlayer.stopPlaying();
        } else {
          if (serial) {
            Serial.println("btn HIGH now");
          }
        }
      }
    }
    lastButtonState = reading;
  }
  delay(100);
}
