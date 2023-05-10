#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <RevEng_PAJ7620.h>

RevEng_PAJ7620 gestureSensor = RevEng_PAJ7620();
Gesture gesture;

#define MAX_CHANNEL 11  // 11 for North America // 13 in Europe

uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

const long runTimeSeconds = 3;  // Seconds to run, if no gestures, before going to sleep

const uint64_t WAKEUP_LOW_PIN_BITMASK = 0b010000;    //GPIO4

typedef struct gestureStruct {
  Gesture gesture;
} gestureStruct;

gestureStruct sendData;

unsigned long lastGestureMillis = millis();

void addPeer(const uint8_t * mac_addr, uint8_t chan){
  esp_now_peer_info_t peer;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void deepSleep(void){
  gpio_set_direction(gpio_num_t(4), GPIO_MODE_INPUT);
  gpio_deep_sleep_hold_dis();
  esp_sleep_config_gpio_isolate();
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_deep_sleep_enable_gpio_wakeup(WAKEUP_LOW_PIN_BITMASK, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}

void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  int i = 10;
  while( !gestureSensor.begin() )             // return value of 1 == success
  {
    i--;
    if (i == 0) {break;}
    Serial.println("PAJ7620 init error -- device not found");
  }

  lastGestureMillis = millis();

}  

void loop() {
  gesture = gestureSensor.readGesture();
  if (gesture == GES_NONE)
  {
    if (millis() - lastGestureMillis >= runTimeSeconds*1000) {
      deepSleep();  //sleep if there are no gestures after waking up
    }
  }
  else {
    Serial.println("Gesture found.");
    sendData.gesture = gesture;
    for (int channel = 1; channel<=MAX_CHANNEL; channel++){
      // change WiFi channel 
      ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
      if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing esp now");
      }
      addPeer(broadcastAddress, channel);
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));
      delay(10);
    }
    lastGestureMillis = millis();
  }
}
