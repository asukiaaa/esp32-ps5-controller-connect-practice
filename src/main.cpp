#include <Arduino.h>

#include "hid_l2cap.h"
#include <BluetoothSerial.h>

// uint8_t TARGET_BT_ADDR[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
uint8_t TARGET_BT_ADDR[] = {0x48, 0x18, 0x8D, 0xA0, 0xBE, 0xC0};

void key_callback(uint8_t *p_msg, size_t len) {
  // do something
  // Serial.println("key_callback");
  // for (int i = 0; i < 8; ++i) {
  //   Serial.print(p_msg[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();
}

void setup() {
  Serial.begin(115200);
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

  // Replace the "1a:2b:3c:01:01:01" with the MAC address
  // the controller wants to pair to
  // Note: MAC address must be unicast
  // ps5.begin("1a:2b:3c:01:01:01");
  Serial.println("Ready.");
  auto ret = hid_l2cap_initialize(key_callback);
  if (ret != 0) {
    while (1) {
      Serial.println("failed initializing l2cap");
      delay(5000);
    }
  }
  ret = hid_l2cap_connect(TARGET_BT_ADDR);
  if (ret != 0) {
    Serial.println("failed connection");
    // while (1) {
    //   delay(5000);
    // }
  }
}

#define BT_CONNECT_TIMEOUT 10000
static uint32_t start_tim;

void loop() {
  BT_STATUS status = hid_l2cap_is_connected();
  Serial.println("status " + String(status));
  if (status == BT_CONNECTING) {
    Serial.println("connecting");
    uint32_t tim = millis();
    if ((tim - start_tim) >= BT_CONNECT_TIMEOUT) {
      start_tim = tim;
      Serial.println("reconnect");
      hid_l2cap_reconnect();
    }
  } else if (status == BT_DISCONNECTED) {
    Serial.println("disconnected");
    start_tim = millis();
    hid_l2cap_reconnect();
  } else if (status == BT_CONNECTED) {
    Serial.println("connected");
  } else {
    Serial.println("other");
  }

  delay(1000);
}
