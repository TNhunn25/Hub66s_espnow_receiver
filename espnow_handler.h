#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include <WiFi.h>
#include <ESP32_NOW.h>
#include "config.h"

extern bool dang_gui; // cờ đang gửi

void onReceive(const esp_now_recv_info *recv_info, const uint8_t *data, int len);

inline void initEspNow()
{

  // Thiết lập chế đọ wifi station
  //  WiFi.mode(WIFI_STA);
  //  delay(100); // Đợi WiFi ổn định
  Serial.println("🌐 WiFi mode set to Station");
  //  Serial.println("MAC Address: " + WiFi.macAddress());

  // Khởi tạo ESP-now
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("❌ ESP-NOW init failed!");
    // networkConnected = false;
    return;
  }
  // networkConnected = true;

  // Callback xử lý trạng thái gửi gói tin
  esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status)
                           {
    Serial.print("Send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "F");
    if (status == ESP_NOW_SEND_SUCCESS) {
      dang_gui = false;
    } });

  // Callback xử lý gói tin nhận được
  esp_now_register_recv_cb([](const esp_now_recv_info *recv_info, const uint8_t *data, int len)
                           { onReceive(recv_info, data, len); });

  // Thêm peer broadcast để nhận gói tin từ mọi thiết bị
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, senderMac, 6); // FF:FF:FF:FF:FF:FF
  peerInfo.channel = 1;                     // Kênh cố định để đồng bộ với sender
  peerInfo.encrypt = false;                 // tạm thời tắt mã hóa
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("❌ Failed to add peer!");
    // networkConnected = false;
  }
  else
    Serial.println("add peer ok");
}

#endif
