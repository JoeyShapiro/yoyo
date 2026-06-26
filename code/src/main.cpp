#include <Arduino.h>
#include <FastLED.h>

#define DATA_PIN    D2
#define NUM_LEDS    16
#define BRIGHTNESS  64
// Change to SK6812 if your strip has a dedicated white channel
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

void rainbow(uint8_t speed) {
  static uint8_t hue = 0;
  fill_rainbow(leds, NUM_LEDS, hue, 255 / NUM_LEDS);
  hue += speed;
}

void colorWipe(CRGB color, uint16_t delayMs) {
  static int i = 0;
  static unsigned long lastMs = 0;
  if (millis() - lastMs >= delayMs) {
    leds[i % NUM_LEDS] = color;
    i++;
    lastMs = millis();
  }
}

void breathe(CRGB color, uint8_t speed) {
  static uint8_t brightness = 0;
  static int8_t direction = 1;
  brightness += direction * speed;
  if (brightness == 0 || brightness >= 250) direction = -direction;
  fill_solid(leds, NUM_LEDS, color);
  FastLED.setBrightness(brightness);
}

void setup() {
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
         .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  // Uncomment one effect at a time:
  rainbow(2);
  // colorWipe(CRGB::Blue, 50);
  // breathe(CRGB::Red, 2);

  FastLED.show();
  delay(20);
}
