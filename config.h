#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <time.h>
#include <MD5Builder.h>
#include "led_status.h"
#include <Preferences.h> //Thư viện lưu trữ dữ liệu không mất khi tắt nguồn

// Định nghĩa chân LED
#define LED_PIN 46

//Cấu hình nhóm ESP- NOW
constexpr uint8_t HUB66S_GROUP_COUNT = 3; //Tổng số nhóm Hub sẽ luân phiên
constexpr uint8_t HUB66S_GROUP_SIZE = 1; //Số lượng Hub tối đa trong mỗi nhóm
constexpr uint8_t HUB66S_GROUP_SLOT_MS = 5000UL; //5 giây cho mỗi slot hoạt động

//Khóa PMK dùng cho mã hóa ESP_NOW 16 byte
static const uint8_t HUB66S_ESPNOW_PMK[16] = {'H', 'u', 'b', '6', '6', 's', 'P', 'm', 'k', 'S', 'e', 'c', 'r', 'e', 't', '!'}; // Khóa PMK dùng chung cho toàn mạng
static const uint8_t HUB66S_ESPNOW_LMK[16] = {'H', 'u', 'b', '6', '6', 's', 'L', 'm', 'k', 'S', 'e', 'c', 'r', 'e', 't', '!'}; // Khóa LMK dùng khi đăng ký peer

// Định nghĩa các opcode
#define LIC_TIME_GET 0x01
#define LIC_SET_LICENSE 0x02
#define LIC_GET_LICENSE 0x03
#define LIC_LICENSE_DELETE 0x04
#define LIC_LICENSE_DELETE_ALL 0x05
#define LIC_INFO 0x06
#define CONFIG_DEVICE 0x07
#define LIC_CONFIG_DEVICE CONFIG_DEVICE
#define LIC_INFO_RESPONSE 0x80

// Kích thước buffer cho JSON
#define BUFFER_SIZE 512

// Địa chỉ MAC broadcast và khóa bí mật
static uint8_t senderMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   // MAC của LIC66S
static uint8_t receiverMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // MAC broadcast

//Khóa bí mật cho hàm băm MD5
#define private_key "khoabi_mat_123"

// Hàm mã hóa Auth MD5
String md5Hash(int id_src, int id_des, String mac_src, String mac_des, uint8_t opcode, const String &data,
               unsigned long timestamp)
{

    MD5Builder md5;
    md5.begin();
    md5.add(String(id_src));
    md5.add(String(id_des));
    md5.add(mac_src);
    md5.add(mac_des);
    md5.add(String(opcode));
    md5.add(data);
    md5.add(String(timestamp));
    md5.add(private_key);
    md5.calculate();
    return md5.toString(); // Trả về chuỗi MD5 hex
}

// Cấu trúc dữ liệu
typedef struct
{
    int lid; // License ID
    int id;
    String license; // Nội dung license
    time_t created;
    time_t expired;
    uint32_t duration;
    uint32_t remain;
    bool expired_flag; // Đã hết hạn chưa
    uint32_t nod;      // number of device
    String deviceName; // Tên thiết bị
    String version;
} LicenseInfo;

typedef struct
{
    char payload[512]; // Kích thước payload có thể điều chỉnh
} PayloadStruct;

// Lấy địa chỉ MAC của thiết bị
String getDeviceMacAddress()
{
    return WiFi.macAddress();
}

// Biến toàn cục
extern LedStatus led;
extern LicenseInfo globalLicense;
extern PayloadStruct message;
extern int config_lid;
extern int config_id; // id_src
extern int id_des;    // id_des
extern bool config_processed;
extern char jsonBuffer[BUFFER_SIZE];
extern int bufferIndex;
extern time_t start_time;
extern const uint32_t duration;
extern bool expired_flag;
extern uint8_t expired;
extern uint32_t now;
extern uint32_t lastSendTime;
extern String device_id;
extern uint32_t nod; // number of device

#endif // CONFIG_H
