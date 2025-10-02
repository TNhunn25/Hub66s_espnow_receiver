#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include <WiFi.h>
#include <ESP32_NOW.h>
#include "config.h"

extern bool dang_gui;          // cờ đang gửi
extern bool waitingSendResult; // Cờ chờ kết quả gửi
extern bool needRetry;         // Cần gửi lại
extern uint8_t retries;        // số lần đã thử gửi lại
extern unsigned long lastTime; // Thời điểm gửi cuối cùng

void onReceive(const esp_now_recv_info *recv_info, const uint8_t *data, int len);

inline void initEspNow()
{
  Serial.println("🌐 WiFi mode set to Station");

  // Khởi tạo ESP-now
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("❌ ESP-NOW init failed!");
    return;
  }

  //Thiết lập khóa PMK dùng chung cho các peer mã hóa
  esp_err_t pmkStatus = esp_now_set_pmk(HUB66S_ESPNOW_PMK);
  if (pmkStatus != ESP_OK)
  {
    Serial.printf("❌ ESP_NOW set failed! err=%d\n", pmkStatus);
  }
  

  /*
  // Callback xử lý trạng thái gửi gói tin
  esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status)
                           {
    Serial.print("Send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "F");
    if (status == ESP_NOW_SEND_SUCCESS) {
      dang_gui = false;
    }
  });
  */  

  // Gắn callback theo dõi kết quả gửi và reset cờ khi thành công
  esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status)
                           {
    Serial.print("Send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "F");
    waitingSendResult = false;
    if (status == ESP_NOW_SEND_SUCCESS)
    {
      dang_gui = false;
      needRetry = false;
      retries = 0;
    }
    else
    {
      needRetry = true;
      lastTime = millis();
    }
  });
  //-----------------------

  // Callback xử lý gói tin nhận được
  esp_now_register_recv_cb([](const esp_now_recv_info *recv_info, const uint8_t *data, int len)
                           { onReceive(recv_info, data, len); });
/*
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
*/

Serial.println("✅ ESP-NOW ready");
}

#endif
