#include "esphome.h"

#if defined(ESP8266)
#pragma message "ESP8266 board found"
#include <ESP8266WiFi.h>
#include <espnow.h>
#elif defined(ESP32)
#pragma message "ESP32 board found"
#include <esp_now.h>
#include <WiFi.h>
#else
#error "ESP8266 or ESP32 boards are required"
#endif

enum Gesture {
  GES_NONE = 0,      /**< No gesture */
  GES_UP,            /**< Upwards gesture */
  GES_DOWN,	         /**< Downward gesture */
  GES_LEFT,          /**< Leftward gesture */
  GES_RIGHT,         /**< Rightward gesture */
  GES_FORWARD,       /**< Forward gesture */
  GES_BACKWARD,      /**< Backward gesture */
  GES_CLOCKWISE,     /**< Clockwise circular gesture */
  GES_ANTICLOCKWISE, /**< Anticlockwise circular gesture */
  GES_WAVE           /**< Wave gesture */
};

Gesture gesture;

int chan; 

bool starting = true;       //We need to init espnow when starting.

typedef struct gestureStruct {
  Gesture gesture;
} gestureStruct;

gestureStruct gestureMessage;

void onDataReceive(const uint8_t * mac_addr, const uint8_t *data, int len) { 

  if (len != sizeof(gestureMessage)) {return;}  // Drop any esp32 broadcasts that are not ours

  memcpy(&gestureMessage, data, sizeof(gestureMessage));

  ESP_LOGD("espnowServer", "Data received from %x. Gesture: %d", mac_addr, gestureMessage.gesture);
  gesture = gestureMessage.gesture;
}

void initEspNow(){
    WiFi.mode(WIFI_AP_STA);
    if (esp_now_init() != ESP_OK) {
      ESP_LOGI("espnowServer", "Error initializing esp now");
      return;
    }
    esp_now_register_recv_cb(onDataReceive);
} 

class GestureSensor : public Component, public TextSensor {
 public:

  void setup() override {
      //Nothing to do in setup
  }

  void loop() override {

    // This doesn't work in setup because ESPHome doesn't init wifi yet.
    if ((WiFi.status() == WL_CONNECTED) && (starting == true)) {
      Serial.println("Wifi connected, starting esp now");
      starting = false;
      chan = WiFi.channel();
      initEspNow();
    }

    if (gesture == GES_UP)
    {
      publish_state("UP");
      gesture = GES_NONE;
    }
    else if(gesture == GES_DOWN)
    {
      publish_state("DOWN");
      gesture = GES_NONE;
    }
    else if(gesture == GES_FORWARD)
    {
      publish_state("FORWARD");
      gesture = GES_NONE;
    }
    else if(gesture == GES_BACKWARD)
    {
      publish_state("BACKWARD");
      gesture = GES_NONE;
    }
    else if(gesture == GES_LEFT)
    {
      publish_state("LEFT");
      gesture = GES_NONE;
    }
    else if(gesture == GES_RIGHT)
    {
      publish_state("RIGHT");
      gesture = GES_NONE;
    }
    else if(gesture == GES_NONE)
    {
    }
  }
};
