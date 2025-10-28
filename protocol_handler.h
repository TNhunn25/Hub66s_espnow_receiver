#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <WiFi.h>
#include <ArduinoJson.h>
#include <MD5Builder.h>
#include "config.h"
#include <Preferences.h>

extern esp_now_recv_info lastRecvInfo;

extern uint8_t lastPacketMac[6];
extern volatile bool hasNewPacket;
extern int lastPacketLen;
extern uint8_t lastPacketData[sizeof(PayloadStruct)];

extern bool dang_gui;          // cờ đang gửi
extern bool waitingSendResult; // Cờ chờ kết quả gửi
extern bool needRetry;         // Cần gửi lại
extern unsigned long lastTime; // Thời điểm gửi cuối cùng

// Cấu trúc cho tin nhắn ESP-NOW
extern PayloadStruct message;

// Khai báo Preferences là extern
extern Preferences preferences;

extern unsigned long runtime;
extern uint32_t group_id; // group indentifier for node queueing

// Chuyển đổi MAC thành String
String macToString(const uint8_t *mac)
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

// Tạo tin nhắn phản hồi
String createMessage(int id_src, int id_des, String mac_src, String mac_des, uint8_t opcode, DynamicJsonDocument data, unsigned long timestamp = 0)
{
    if (timestamp == 0)
    {
        timestamp = millis() / 1000; // Mô phỏng thời gian Unix
    }
    String dataStr;
    serializeJson(data, dataStr);                                                        // Chuyển dữ liệu thành chuỗi JSON
    String auth = md5Hash(id_src, id_des, mac_src, mac_des, opcode, dataStr, timestamp); // Tạo mã MD5

    DynamicJsonDocument jsonDoc(512);
    jsonDoc["id_src"] = id_src; // ID nguồn
    jsonDoc["id_des"] = id_des; // ID đích
    // jsonDoc["mac_src"] = mac_src; // MAC nguồn
    // jsonDoc["mac_des"] = mac_des; // MAC đích
    jsonDoc["opcode"] = opcode;  // Opcode
    jsonDoc["data"] = data;      // Dữ liệu
    jsonDoc["time"] = timestamp; // Thời gian
    jsonDoc["auth"] = auth;      // Mã xác thực

    String messageStr;
    serializeJson(jsonDoc, messageStr); // Chuyển thành chuỗi JSON
    return messageStr;
}

// Gửi phản hồi
void sendResponse(int id_src, int id_des, String mac_src, String mac_des, uint8_t opcode, const DynamicJsonDocument &data, const uint8_t *targetMac)
{
    String targetMacStr = macToString(targetMac);                                       // Chuyển đổi targetMac thành chuỗi
    String output = createMessage(id_src, id_des, mac_src, targetMacStr, opcode, data); // Sử dụng targetMacStr làm mac_des
    if (output.length() > sizeof(message.payload))
    {
        // Serial.println("❌ Payload quá lớn!");
        Serial.printf("❌ Payload quá lớn (%u > %u), không gửi được\n", output.length(), sizeof(message.payload));
        led.setState(CONNECTION_ERROR);
        dang_gui = false;
        return;
    }
    output.toCharArray(message.payload, sizeof(message.payload)); // Chuyển vào payload

    //---------------
    dang_gui = true;
    waitingSendResult = true;
    needRetry = false;
    lastTime = millis();
    //---------------

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, targetMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    if (!esp_now_is_peer_exist(targetMac))
    {
        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            Serial.println("❌ Failed to add peer!");
            return;
        }
        Serial.println("Đã thêm peer");
        delay(100); // Đợi một chút để đảm bảo peer đã được thêm
    }

    // esp_now_send(targetMac, (uint8_t *)&message, sizeof(message)); // Gửi qua ESP-NOW
    esp_err_t sendResult = esp_now_send(targetMac, (uint8_t *)&message, sizeof(message)); // Gửi qua ESP-NOW
    Serial.println("\n📤 Đã gửi phản hồi:");
    Serial.println(output);

    //---------------
    if (sendResult != ESP_OK)
    {
        Serial.printf("❌ Lỗi gửi ESP-NOW: %d\n", sendResult);
        waitingSendResult = false;
        needRetry = true;
    }
    //---------------

    // Xóa peer sau khi gửi
    if (esp_now_is_peer_exist(targetMac))
    {
        esp_now_del_peer(targetMac);
    }
    else
    {
        Serial.println("Peer không tồn tại");
    }
}

// Lưu dữ liệu license vào NVS (Non-Volatile Storage)
// NVS: đảm bảo dữ liệu không bị mất khi thiết bị tắt nguồn.
void saveLicenseData(bool verbose = true)
{
    preferences.begin("license", false);
    preferences.putInt("lid", globalLicense.lid);
    preferences.putInt("id", globalLicense.id);
    preferences.putULong("created", globalLicense.created);
    preferences.putInt("duration", globalLicense.duration);
    preferences.putInt("remain", globalLicense.remain);
    preferences.putBool("expired_flag", globalLicense.expired_flag);
    preferences.putULong("runtime", runtime);
    preferences.putUInt("group_id", globalLicense.group_id);
    preferences.putULong("last_save", millis() / 1000);
    preferences.end();
    if (verbose)
    {
        Serial.println("✅ Đã lưu dữ liệu license vào NVS");
        Serial.print("Expired: ");
        Serial.println(globalLicense.expired_flag ? 1 : 0);
        Serial.print("Remain: ");
        Serial.println(globalLicense.remain);
    }
}

// Lưu cấu hình thiết bị
void saveDeviceConfig()
{
    preferences.begin("license", false);
    // Lưu cấu hình thiết bị với kiểu dữ liệu nhất quán
    preferences.putInt("config_lid", config_lid);
    preferences.putInt("config_id", config_id);
    preferences.putInt("group_id", ::group_id);
    preferences.end();
}

// Tải dữ liệu license từ NVS (gọi trong setup())
void loadLicenseData()
{
    preferences.begin("license", true); // Mở namespace "license" ở chế độ read-only
    globalLicense.lid = preferences.getInt("lid", 0);
    globalLicense.id = preferences.getInt("id", config_id);
    config_lid = preferences.getInt("config_lid", config_lid);
    config_id = preferences.getInt("config_id", config_id);
    globalLicense.created = preferences.getULong("created", 0);
    globalLicense.duration = preferences.getInt("duration", 0);
    globalLicense.remain = preferences.getInt("remain", 0);
    globalLicense.expired_flag = preferences.getBool("expired_flag", false);
    runtime = preferences.getULong("runtime", 0);
    globalLicense.group_id = preferences.getUInt("group_id", 0); // Bổ sung: Đọc NOD, mặc định 10
    ::group_id = globalLicense.group_id;
    preferences.end();

    Serial.println("✅ Đã đọc và cập nhật dữ liệu license từ NVS:");
    Serial.print("LID: ");
    Serial.println(globalLicense.lid);
    Serial.print("Expired: ");
    Serial.println(globalLicense.expired_flag ? 1 : 0);
    // Serial.print("Remain: "); Serial.println(globalLicense.remain);
    Serial.print("Runtime: ");
    Serial.println(runtime);
}
void onReceive(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len)
{
    // // Copy nguyên struct
    // lastRecvInfo = *recv_info;
    // // Copy payload (giới hạn kích thước)
    // lastPacketLen = min(len, (int)sizeof(lastPacketData));
    // memcpy(lastPacketData, incomingData, lastPacketLen);
    // // Đánh dấu có gói mới
    // hasNewPacket = true;

    // Copy nguyên struct
    lastRecvInfo = *recv_info;
    // Sao lưu MAC nguồn vì con trỏ trong recv_info có thể không còn hợp lệ
    memcpy(lastPacketMac, recv_info->src_addr, sizeof(lastPacketMac));
    // Copy payload (giới hạn kích thước)
    lastPacketLen = min(len, (int)sizeof(lastPacketData));
    memcpy(lastPacketData, incomingData, lastPacketLen);
    // Đánh dấu có gói mới
    hasNewPacket = true;
}

// Xử lý dữ liệu nhận được
void xu_ly_data(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len)
{
    const uint8_t *mac_addr = lastPacketMac;
    (void)recv_info; // đã sao lưu MAC nên tránh cảnh báo biến không dùng
    // const uint8_t *mac_addr = recv_info->src_addr;
    String myMac = WiFi.macAddress();
    time_t now = time(nullptr);
    Serial.println("\n📩 Nhận package tin:");

    // Phân tích JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, incomingData, len);
    if (error)
    {
        Serial.print("❌ Lỗi giải mã JSON: ");
        Serial.println(error.c_str());
        DynamicJsonDocument respDoc(256);
        respDoc["status"] = 255;
        sendResponse(0, 0, myMac, "FF:FF:FF:FF:FF:FF", 0, respDoc, mac_addr);
        led.setState(CONNECTION_ERROR);
        return;
    }

    // Lấy các trường từ gói tin
    int id_src = doc["id_src"].as<int>();         // ID của thiết bị gửi
    int id_des = doc["id_des"].as<int>();         // ID của thiết bị nhận
    String mac_src = doc["mac_src"].as<String>(); // MAC của thiết bị gửi
    String mac_des = doc["mac_des"].as<String>(); // MAC của thiết bị nhận
    uint8_t opcode = doc["opcode"].as<uint8_t>();
    String dataStr;
    serializeJson(doc["data"], dataStr);
    unsigned long packetTime = doc["time"].as<long>();
    String receivedAuth = doc["auth"].as<String>();

    // Bỏ qua gói không dành cho thiết bị
    // if (id_des != config_id && id_des != 0 && mac_des != myMac && mac_des != "FF:FF:FF:FF:FF:FF")
    // if (id_des != config_id && id_des != 0)
    // {
    //     Serial.println("❌ Gói tin không dành cho thiết bị này!");
    //     return;
    // }

    // Bỏ qua gói không dành cho thiết bị trừ khi đây là gói cấu hình
    const bool targetsDevice = (id_des == config_id || id_des == 0);
    const bool isConfigRequest = (opcode == CONFIG_DEVICE);
    if (!targetsDevice && !isConfigRequest)
    {
        Serial.println("❌ Gói tin không dành cho thiết bị này!");
        return;
    }

    // Kiểm tra xác thực MD5
    String calculatedAuth = md5Hash(id_src, id_des, mac_src, mac_des, opcode, dataStr, packetTime);
    if (!receivedAuth.equalsIgnoreCase(calculatedAuth))
    {
        DynamicJsonDocument respDoc(256);
        respDoc["status"] = 1;
        sendResponse(config_id, id_src, myMac, mac_src, opcode | 0x80, respDoc, mac_addr);
        Serial.println("❌ Xác thực MD5 thất bại!");
        led.setState(CONNECTION_ERROR);
        return;
    }
    Serial.println("✅ Xác thực MD5 thành công!");

    // In thông tin gói tin
    Serial.print("From MAC: ");
    for (int i = 0; i < 6; i++)
    {
        Serial.printf("%02X", mac_addr[i]);
        if (i < 5)
            Serial.print(":");
    }
    Serial.println();
    // test
    Serial.println("Device ID: " + String(config_id));
    Serial.println("Local ID: " + String(globalLicense.lid));
    Serial.println("Full received packet:");

    Serial.print("Opcode: 0x0");
    Serial.println(opcode, HEX);
    serializeJsonPretty(doc, Serial);
    Serial.println();

    // Xử lý theo opcode
    switch (opcode)
    {
    case LIC_SET_LICENSE:
    {
        JsonObject data = doc["data"].as<JsonObject>();
        int lid = data["lid"].as<int>(); // Changed from String to int
        int id = data["id"].as<int>();   // Changed from String to int
        time_t created = data["created"].as<long>();
        int duration = data["duration"].as<int>();
        int expired = data["expired"].as<int>(); // kiểm tra biến nếu đúng license còn hiệu lực set_lic và phản hồi.
        int groupId = data["group_id"].as<int>();
        // Nếu sai thì phản hồi lại lic hết hiệu lực.

        DynamicJsonDocument respDoc(256);
        bool isValid = false;
        String error_msg = "";

        // Kiểm tra LID và ID theo yêu cầu
        if (lid != 0 && lid == config_lid)
        { // LID phải khác 0 và khớp với config_lid
            if (id == 0 || id == config_id)
            {                   // ID có thể là 0 hoặc khớp với config_id
                isValid = true; //  ID khớp với thiết bị hoặc ID = 0
            }
            else
            {
                error_msg = "ID không dành cho thiết bị này"; // ID sai
            }
        }
        else
        {
            error_msg = "LID không hợp lệ"; // LID sai
        }
        if (isValid)
        {
            if (expired)
            {
                globalLicense.created = created;
                globalLicense.duration = duration;
                globalLicense.group_id = groupId; // cập nhật group id
                start_time = millis() / 1000;     // đánh dấu mốc thời gian mới
                runtime = 0;
                globalLicense.remain = duration;    // làm mới thời gian còn lại
                globalLicense.expired_flag = false; // chắc chắn đánh dấu
                saveLicenseData();

                // Phản hồi thành công
                respDoc["lid"] = lid;
                respDoc["group_id"] = globalLicense.group_id; // trả về group ID đã cấu hình
                respDoc["status"] = 0;                        // Thành công
                sendResponse(config_id, id_src, myMac, mac_src, LIC_SET_LICENSE | 0x80, respDoc, mac_addr);
                Serial.println("✅ Cập nhật giấy phép thành công: LID = " + String(lid) + ", ID = " + String(id));
                led.setState(FLASH_TWICE); // chớp LED 3 lần để xác nhận
                while (led.isBusy())
                {
                    delay(10); // Chờ LED hoàn thành chớp
                }
                saveLicenseData();
                delay(100);
            }
            else
            {
                respDoc["status"] = 3; // license hết hạn
                respDoc["group_id"] = globalLicense.group_id;
                sendResponse(config_id, id_src, myMac, mac_src, LIC_SET_LICENSE | 0x80, respDoc, mac_addr);
                Serial.println("❌ Giấy phép hết hiệu lực");
                led.setState(CONNECTION_ERROR);
            }
        }
        else
        {
            // sai thông tin LID hoặc ID
            respDoc["status"] = 1; // LID hoặc ID không hợp lệ
            respDoc["error_msg"] = error_msg;
            Serial.println("❌ Lỗi: " + error_msg + " (LID = " + String(lid) + ", ID = " + String(id) + ", config_id = " + String(config_id) + ")");
            sendResponse(config_id, id_src, myMac, mac_src, LIC_SET_LICENSE | 0x80, respDoc, mac_addr);
            led.setState(CONNECTION_ERROR);
        }
        break;
    }

    case LIC_GET_LICENSE:
    {
        JsonObject data = doc["data"].as<JsonObject>();
        uint32_t lid = data["lid"].as<uint32_t>(); // Changed from String to int
        DynamicJsonDocument respDoc(512);

        // Đồng bộ group_id nếu node gửi kèm cấu hình mới
        bool hasGroupIdField = data.containsKey("group_id");
        if (hasGroupIdField)
        {
            uint32_t requestedGroupId = data["group_id"].as<uint32_t>();
            if (requestedGroupId <= 255)
            {
                bool groupChanged = (globalLicense.group_id != requestedGroupId) ||
                                    (static_cast<uint32_t>(::group_id) != requestedGroupId) ||
                                    (config_group_id < 0 || static_cast<uint32_t>(config_group_id) != requestedGroupId);

                if (groupChanged)
                {
                    ::group_id = requestedGroupId;
                    globalLicense.group_id = requestedGroupId;
                    config_group_id = static_cast<int>(requestedGroupId);
                    saveDeviceConfig();
                    saveLicenseData(false);
                    Serial.printf("✅ Đồng bộ group_id từ node: %u\n", requestedGroupId);
                }
            }
            else
            {
                Serial.printf("⚠️ Group ID không hợp lệ trong request: %u\n", requestedGroupId);
            }
        }

        // Kiểm tra LID có hợp lệ không
        if (lid == config_lid || lid == 0)
        {
            respDoc["lid"] = globalLicense.lid;
            respDoc["created"] = globalLicense.created;
            respDoc["expired"] = globalLicense.expired_flag ? 1 : 0;
            respDoc["duration"] = globalLicense.duration;
            respDoc["remain"] = globalLicense.remain;
            if (config_group_id >= 0)
            {
                respDoc["group_id"] = config_group_id;
            }
            else
            {
                respDoc["group_id"] = globalLicense.group_id;
            }
            respDoc["status"] = 0; // Thành công
            Serial.println("✅ License info sent for LID = " + String(lid));
            sendResponse(config_id, id_src, myMac, mac_src, LIC_GET_LICENSE | 0x80, respDoc, mac_addr);
            led.setState(FLASH_TWICE); // chớp xác nhận truy vấn OK

            // Đợi LED chớp xong rồi hiển  thị trạng thái License
            while (led.isBusy())
            {
                led.update(); // duy trì hiệu ứng nhấp nháy
                delay(10);    // Chờ LED hoàn thành chớp
            }
            // Sau khi chớp xong, hiển thị trạng thái License
            if (globalLicense.expired_flag || globalLicense.remain <= 0)
            {
                Serial.println(F("🔒 License đã HẾT HẠN!"));
                led.setState(LICENSE_EXPIRED); // LED tắt
            }
            else
            {
                Serial.printf("🔓 License còn %lu phút\n", globalLicense.remain);
                led.setState(NORMAL_STATUS); // LED sáng liên tục
            }
        }
        else
        {
            Serial.println("❌ LID không hợp lệ: " + String(lid));
            led.setState(CONNECTION_ERROR);
        }
        break;
    }

    case CONFIG_DEVICE:
    {
        JsonObject data = doc["data"].as<JsonObject>();

        int lid = 0;
        if (data.containsKey("new_lid"))
        {
            lid = data["new_lid"].as<int>();
        }
        else if (data.containsKey("lid"))
        {
            lid = data["lid"].as<int>();
        }

        int id = 0;
        if (data.containsKey("new_id"))
        {
            id = data["new_id"].as<int>();
        }
        else if (data.containsKey("id"))
        {
            id = data["id"].as<int>();
        }

        uint32_t requestedNod = ::group_id;
        if (data.containsKey("group_id"))
        {
            requestedNod = data["group_id"].as<uint32_t>();
        }

        DynamicJsonDocument respDoc(384);
        JsonObject responseHint = respDoc.createNestedObject("response_hint");
        // Thông báo cho node biết Hub mong đợi group_id trong phản hồi kế tiếp
        responseHint["expect_group_id"] = true;
        // Giải thích chính sách dự phòng: nếu thiếu group_id thì dùng lại cấu hình trên Hub
        responseHint["fallback_policy"] = "reuse_configured_assignment";
        responseHint["applies_to"] = "existing_and_new"; // áp dụng cho cả thiết bị đang hoạt động và mới gia nhập
        bool isValid = true;
        String error_msg; // thông báo lỗi

        // Kiểm tra LID và ID và số lượng thiết bị (NOD)
        if (lid == 0)
        {
            isValid = false;
            error_msg = "LID không hợp lệ";
        }
        else if (id == 0)
        {
            isValid = false;
            error_msg = "ID không hợp lệ";
        }

        // Mặc định sử dụng group_id hiện tại của Hub nếu request không gửi group_id mới
        int requestedGroupId = config_group_id;
        bool hasGroupId = false;
        if (data.containsKey("group_id"))
        {
            requestedGroupId = data["group_id"].as<int>();
            hasGroupId = true;
        }

        if (hasGroupId && (requestedGroupId < 0 || requestedGroupId > 255))
        {
            isValid = false;
            error_msg = "Group ID không hợp lệ";
        }

        if (!hasGroupId)
        {
            // Ghi log để kỹ thuật viên biết Hub sẽ giữ nguyên cấu hình nhóm đang có
            Serial.println("ℹ️ Không nhận được group_id mới, Hub sẽ giữ nguyên cấu hình hiện có cho cả thiết bị cũ và mới.");
        }

        if (isValid)
        {
            // Cập nhật cấu hình thiết bị
            config_lid = lid;
            config_id = id;
            ::group_id = requestedGroupId;
            globalLicense.group_id = requestedGroupId;

            if (hasGroupId)
            {
                config_group_id = requestedGroupId;
            }

            saveDeviceConfig();
            // lưu vào NVS
            saveLicenseData(false); // lưu trạng thái license hiện tại nhưng không in log

            // chớp LED để xác nhận
            led.setState(FLASH_TWICE); // chớp 3 lần
            Serial.println("✅ Cấu hình thành công: LID = " + String(lid) + ", ID = " + String(id) + "GROUP ID = " + String(requestedGroupId));

            if (config_group_id >= 0)
            {
                Serial.printf("✅ Nhóm được gán: %d\n", config_group_id);
            }

            respDoc["status"] = 0;
            respDoc["lid"] = config_lid;
            respDoc["id"] = config_id;
            respDoc["group_id"] = ::group_id;

            if (config_group_id >= 0)
            {
                // Gửi kèm group_id đã gán để node đồng bộ cấu hình với Hub
                respDoc["group_id"] = config_group_id;
                responseHint["group_id"] = config_group_id;
            }
        }
        else
        {
            respDoc["status"] = 1;
            respDoc["error_msg"] = error_msg;
            Serial.println("❌ Lỗi: " + error_msg + " (LID = " + String(lid) + ", ID = " + String(id) + ", config_id = " + String(config_id) + ")");
            led.setState(CONNECTION_ERROR);
            if (config_group_id >= 0)
            {
                // Khi lỗi vẫn nhắc lại group_id hiện hành để node nắm được cấu hình Hub đang duy trì
                responseHint["group_id"] = config_group_id;
            }
        }

        sendResponse(config_id, id_src, myMac, mac_src, CONFIG_DEVICE | 0x80, respDoc, mac_addr);
        break;
    }

    case LIC_LICENSE_DELETE:
    {
        int lid = doc["data"]["lid"].as<int>();
        DynamicJsonDocument respDoc(256);
        respDoc["lid"] = lid;
        respDoc["status"] = (lid == globalLicense.lid) ? 0 : 3; // Thành công hoặc không tìm thấy
        if (lid == globalLicense.lid)
        {
            // Xóa license bằng cách đặt về giá trị mặc định
            globalLicense.lid = 0;
            globalLicense.created = 0;
            globalLicense.duration = 0;
            globalLicense.remain = 0;
            globalLicense.expired_flag = false;
            saveLicenseData(); // Lưu trạng thái mới
        }
        sendResponse(config_id, id_src, myMac, mac_src, LIC_LICENSE_DELETE | 0x80, respDoc, mac_addr);
        Serial.println(lid == globalLicense.lid ? "✅ Đã xóa license cho LID = " + String(lid) : "❌ Không tìm thấy license cho LID = " + String(lid));
        led.setState(lid == globalLicense.lid ? NORMAL_STATUS : CONNECTION_ERROR);
        break;
    }

    case LIC_LICENSE_DELETE_ALL:
    {
        // Xóa tất cả license
        globalLicense.lid = 0;
        globalLicense.created = 0;
        globalLicense.duration = 0;
        globalLicense.remain = 0;
        globalLicense.expired_flag = false;
        saveLicenseData(); // Lưu trạng thái mới
        DynamicJsonDocument respDoc(256);
        respDoc["status"] = 0; // Thành công
        sendResponse(config_id, id_src, myMac, mac_src, LIC_LICENSE_DELETE_ALL | 0x80, respDoc, mac_addr);
        Serial.println("✅ Đã xóa tất cả license.");
        led.setState(NORMAL_STATUS);
        break;
    }

    case LIC_TIME_GET:
    {
        DynamicJsonDocument respDoc(256);
        respDoc["time"] = millis() / 1000; // Trả về thời gian receiver
        respDoc["status"] = 0;
        sendResponse(config_id, id_src, myMac, mac_src, LIC_TIME_GET | 0x80, respDoc, mac_addr);
        Serial.println("✅ Time info sent.");
        break;
    }

    case LIC_INFO:
    {
        DynamicJsonDocument respDoc(256);
        respDoc["deviceName"] = globalLicense.deviceName;
        respDoc["version"] = globalLicense.version;
        respDoc["status"] = 0;
        sendResponse(config_id, id_src, myMac, mac_src, LIC_INFO_RESPONSE, respDoc, mac_addr);
        Serial.println("✅ Device info sent.");
        break;
    }

    default:
    {
        DynamicJsonDocument respDoc(256);
        respDoc["status"] = 255; // Opcode không xác định
        sendResponse(config_id, id_src, myMac, mac_src, opcode | 0x80, respDoc, mac_addr);
        Serial.printf("❌ Unknown opcode: 0x%02X\n", opcode);
        led.setState(CONNECTION_ERROR);
        break;
    }
    }
}
#endif // PROTOCOL_HANDLER_H
