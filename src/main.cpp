/*********************************************************************
MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <ESP32Encoder.h>
#include <string>
#include <algorithm>
#include <vector>
#include <Wire.h>
#include "Adafruit_NeoTrellis.h"
#include "main.h"

#define I2C_SDA 47
#define I2C_SCL 48

Adafruit_NeoTrellis trellis = Adafruit_NeoTrellis();

const int encoder_no = 2;
ESP32Encoder encoders[encoder_no];
int8_t pins[encoder_no*2] = {40,41,35,36};
int64_t prev_enc_val[encoder_no];

const int slider_no = 2;
int8_t slider_pins[slider_no] = {9, 10};
int64_t prev_slider_val[slider_no];

// USB WebUSB object
Adafruit_USBD_WebUSB usb_web;

// Landing Page: scheme (0: http, 1: https), url
// Page source can be found at https://github.com/hathach/tinyusb-webusb-page/tree/main/webusb-serial
WEBUSB_URL_DEF(landingPage, 1 /*https*/, "example.tinyusb.org/webusb-serial/index.html");

void line_state_callback(bool connected)
{
  if ( connected )
  {
    usb_web.println("WebUSB interface connected !!");
    usb_web.flush();
  }
}

std::vector<char> cmd;
const size_t bufsize = 128;
char buf[bufsize];

// the setup function runs once when you press reset or power the board
void setup()
{
  cmd.reserve(100);
  Wire.setPins(I2C_SDA, I2C_SCL);
  for(int i = 0; i < encoder_no; i++) {
    encoders[i].attachHalfQuad(pins[i*2], pins[i*2+1]);
    encoders[i].setCount(0);
  }

  if (!trellis.begin()) {
    Serial.println("Could not start trellis");
    while(1) {
      Serial.println("Could not start trellis");
      delay(1);
    };
  }

  //activate all keys and set callbacks
  for(int i=0; i<NEO_TRELLIS_NUM_KEYS; i++){
    trellis.activateKey(i, SEESAW_KEYPAD_EDGE_RISING);
    trellis.activateKey(i, SEESAW_KEYPAD_EDGE_FALLING);
    trellis.registerCallback(i, blink);
  }
  
  usb_web.setLandingPage(&landingPage);
  usb_web.setLineStateCallback(line_state_callback);
  usb_web.setStringDescriptor("TinyUSB WebUSB");
  usb_web.begin();

  Serial.begin(115200);

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
}

uint32_t count;

void loop()
{
  handle_enc();
  handle_sliders();
  trellis.read();

  // from WebUSB to both Serial & webUSB
  if (usb_web.available())
  {
    count = usb_web.readBytes(buf, bufsize);
    for(int i = 0; i < count; ++i) {
      if(buf[i] == '\r') {
        HandleCommand(std::string_view(cmd.data(), cmd.size()));
        cmd.clear();
      } else {
        cmd.push_back(buf[i]);
      }
    }
  }
}

void HandleCommand(std::string_view &&cmd) {
  std::string type = std::string(cmd.substr(0, std::max(cmd.find("."), (size_t)0)));
  std::string target = std::string(cmd.substr(cmd.find(".") + 1, std::max(cmd.find("-") - cmd.find(".") - 1, (size_t)0)));
  std::string data = std::string(cmd.substr(cmd.find("-") + 1, std::max(cmd.length() - cmd.find("-") - 1, (size_t)0)));
  usb_web.println(("Handling command: " + type + " // " + target + " // " + data).c_str());
  usb_web.flush();
}


void handle_enc() {
  bool encoder_changed = false;
  for(int i = 0; i < encoder_no; i++) {
    int64_t enc_val = encoders[i].getCount();
    if(enc_val != prev_enc_val[i]) {
      encoder_changed = true;
      usb_web.println(("ENC." + std::to_string(i) + " - " + std::to_string(enc_val) + ", ").c_str());
    }

    prev_enc_val[i] = enc_val;
  }

  if(encoder_changed) {
    usb_web.flush();
  }
}

void handle_sliders() {
  bool slider_changed = false;
  for(int i = 0; i < slider_no; i++) {
    int slider_val = analogRead(slider_pins[i]);
    if(slider_val != prev_slider_val[i]) {
      slider_changed = true;
      usb_web.println(("SLDR." + std::to_string(i) + " - " + std::to_string(slider_val)).c_str());
    }
  }
  if(slider_changed) {
    usb_web.flush();
  }
}

TrellisCallback blink(keyEvent evt) {
  // Check is the pad pressed?
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    trellis.pixels.setPixelColor(evt.bit.NUM, Wheel(map(evt.bit.NUM, 0, trellis.pixels.numPixels(), 0, 255))); //on rising
  } else if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_FALLING) {
  // or is the pad released?
    trellis.pixels.setPixelColor(evt.bit.NUM, 0); //off falling
  }

  usb_web.println(("TREKEY." + std::to_string(evt.bit.NUM) + "-" + std::to_string(evt.bit.EDGE)).c_str());
  usb_web.flush();
  // Turn on/off the neopixels!
  trellis.pixels.show();

  return 0;
}

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return trellis.pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return trellis.pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return trellis.pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  return 0;
}