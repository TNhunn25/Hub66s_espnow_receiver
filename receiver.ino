#include <WiFi.h>
#include <ArduinoJson.h>

#include "config.h"
#include "led_status.h"
#include "espnow_handler.h"
#include "protocol_handler.h"
#include "serial.h"
#include "watch_dog.h"
#include "led_display.h"

// Define Preferences object
Preferences preferences;

LedStatus led(LED_PIN, /*activeHigh=*/false); // nếu LED nối kiểu active-LOW
Hub66s::LedDisplay ledDisplay;                // Khởi tạo đối tượng từ lớp LedDisplay

// Cấu trúc dữ liệu License
LicenseInfo globalLicense;
PayloadStruct message;

// Biến lưu cấu hình
int config_lid = 115;
int config_id = 2011; // ID của HUB66S
int id_des = 1001;    // ID của LIC66S
String device_id = "HUB66S_001";

bool config_processed = false;
char jsonBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool expired_flag = false; // Cờ hết hạn
uint8_t expired = 0;       // Biến lưu trạng thái hết hạn
uint32_t now;
time_t start_time = 0;          // thời điểm bắt đầu tính thời gian
const uint32_t duration = 60;   // Giá trị cố định sau khi gán lần đầu cho license
uint32_t lastSendTime = 0;      // Thời điểm gửi gói tin gần nhất
uint32_t lastRuntimeUpdate = 0; // Thời điểm cập nhật runtime gần nhất
// uint32_t lastPacketNodeId = 0; // Chỉ ở một chỗ duy nhất!

// bool networkConnected = false;
uint32_t runtime = 0;
uint32_t nod = 0; // số lượng thiết bị giả định 10
bool dang_gui = false; // cờ đang gửi
uint32_t lastTime = 0; // thời điểm gửi lần cuối
uint8_t retries = 0;   // số lần đã thử gửi

uint8_t lastPacketData[512]; // cũ 250 Lưu payload (có thể điều chỉnh kích thước tuỳ theo nhu cầu, ở đây bằng tối đa của PayloadStruct)
esp_now_recv_info lastRecvInfo;

// Lưu MAC của gói vừa nhận
uint8_t lastPacketMac[6];
volatile bool hasNewPacket = false; // Lưu độ dài payload
int lastPacketLen;
// Lưu payload (có thể điều chỉnh kích thước tuỳ theo nhu cầu, ở đây bằng tối đa của PayloadStruct)

void xu_ly_dang_gui()
{
  // Chỉ xử lý khi đang trong trạng thái gửi
  if (!dang_gui)
    return;

  uint32_t now = millis();
  // Chưa đủ 1s kể từ lần gửi trước thì bỏ qua
  if (now - lastTime < 1000)
    return;

  // Đã đủ 1s, cập nhật thời điểm và thử gửi
  lastTime = now;
  xu_ly_data(&lastRecvInfo, lastPacketData, lastPacketLen);
  Serial.println("📤 Đang gửi gói tin...");
  // gọi hàm truyền data
  // Gửi thất bại, tăng bộ đếm và thử lại
  retries++;
  if (retries >= 3)
  {
    // Đã thử 3 lần mà vẫn fail → dừng gửi
    dang_gui = false;
    retries = 0;
  }
}
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n🌟 HUB66S Receiver Started");
  ledDisplay.begin();          // Khởi tạo module LED hiển thị
  Hub66s::WatchDog::begin(10); // ⭐ khởi tạo WDT 10 s (toàn chip reset khi quá hạn)

  WiFi.mode(WIFI_STA); // Enable Wi-Fi in Station mode for ESP-NOW
  delay(100);
  // WiFi.setTxPower(WIFI_POWER_2dBm);
  initEspNow();                     // Initialize ESP-NOW
  configTime(0, 0, "pool.ntp.org"); // Configure NTP for time synchronization

  esp_now_register_recv_cb(onReceive); // Register receive callback

  loadLicenseData();
  // Biến trả về giá trị của node
  globalLicense.lid = config_lid;
  globalLicense.id = config_id;
  globalLicense.nod = nod;

  // Khởi tạo trạng thái expired dựa trên globalLicense
  if (globalLicense.remain > 0 && !globalLicense.expired_flag)
  {
    expired = 0; // Giấy phép còn hạn
  }
  else
  {
    expired = 1; // Giấy phép hết hạn hoặc không hợp lệ
  }
  saveLicenseData();
  led.setState(CONNECTION_ERROR);
}

void loop()
{
  recPC();
  serialPC();
  led.update();
  ledDisplay.update(); // Cập nhật hiển thị màn hình LED
  delay(10);           // Giảm tải CPU

  // Kiểm tra license và gửi thông tin định kỳ
  uint32_t nowMillis = millis();
  if (nowMillis - lastRuntimeUpdate >= 60000)
  {
    lastRuntimeUpdate = nowMillis;
    now = time(nullptr);

    if (globalLicense.lid != 0 && globalLicense.duration > 0)
    {
      runtime++; // tăng thời gian chạy từng phút
      preferences.begin("license", false);
      preferences.putULong("runtime", runtime);
      preferences.end();
      globalLicense.remain = globalLicense.duration > runtime ? globalLicense.duration - runtime : 0; // Ngăn remain âm

      // Kiểm tra license hết hạn
      if (globalLicense.remain <= 0 && !globalLicense.expired_flag)
      {
        globalLicense.expired_flag = true;
        globalLicense.remain = 0;
        expired = 1; // Giấy phép hết hạn
        saveLicenseData(false);
      }
      else if (globalLicense.remain > 0 && globalLicense.expired_flag)
      {
        globalLicense.expired_flag = false;
        expired = 0; // Giấy phép còn hạn
        saveLicenseData(false);
      }
      else
      {
        saveLicenseData(false);
      }
    }
    else
    {
      // Giấy phép không hợp lệ
      expired = 1; // Hết hạn
      globalLicense.expired_flag = true;
      globalLicense.remain = 0;
      saveLicenseData(false);
    }
    // Cập nhật LED trạng thái
    if (globalLicense.expired_flag || globalLicense.remain <= 0)
    {
      led.setState(LICENSE_EXPIRED); // LED tắt
    }
    else
    {
      led.setState(NORMAL_STATUS); // LED sáng liên tục
    }
  }
  if (hasNewPacket)
  {
    // Gọi hàm xử lý với đúng kiểu
    xu_ly_data(&lastRecvInfo, lastPacketData, lastPacketLen);
    // Reset cờ
    hasNewPacket = false;
    dang_gui = true;
    retries = 0;
  }
  xu_ly_dang_gui();

  Hub66s::WatchDog::feed(); // Reset WDT timer
  // delay(100); // Giảm tải CPU
}
