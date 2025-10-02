#ifndef HUB66S_ESPNOW_SCHEDULER_H
#define HUB66S_ESPNOW_SCHEDULER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <algorithm>
#include <cstring>
#include <iterator>

namespace Hub66s
{
    // Đại diện cho một thiết bị peer cần được Hub quản lý
    struct PeerDevice
    {
        uint8_t mac[6];   // Địa chỉ MAC của thiết bị peer
        bool encrypt;     // Cờ cho biết peer có dùng mã hóa LMK hay không
        uint8_t lmk[16];  // Khóa LMK 16 byte (chỉ sử dụng khi encrypt = true)
        uint8_t channel;  // Kênh Wi-Fi mà peer hoạt động

        PeerDevice()
        {
            // Khởi tạo mặc định với MAC & LMK bằng 0, không bật mã hóa và kênh 1
            std::fill(std::begin(mac), std::end(mac), 0x00);
            std::fill(std::begin(lmk), std::end(lmk), 0x00);
            encrypt = false;
            channel = 1;
        }
    };

    // Bộ lập lịch luân phiên peer ESP-NOW theo nhóm
    class EspNowScheduler
    {
    public:
        static constexpr uint8_t kGroupCount = 5;        // Tổng số nhóm hoạt động
        static constexpr uint8_t kGroupSize = 2;        // Số peer tối đa trong mỗi nhóm
        static constexpr uint32_t kDefaultSlotMs = 5000; // Thời lượng mặc định cho một slot

        EspNowScheduler()
            : activeGroup_(kGroupCount), slotDurationMs_(kDefaultSlotMs), lastSwitchMs_(0), initialized_(false)
        {
            // Ban đầu tất cả các nhóm đều rỗng
            for (uint8_t group = 0; group < kGroupCount; ++group)
            {
                groups_[group].count = 0;
            }
        }

        void setSlotDuration(uint32_t durationMs)
        {
            // Bảo vệ tránh truyền 0 → dùng lại giá trị mặc định
            slotDurationMs_ = durationMs == 0 ? kDefaultSlotMs : durationMs;
        }

        void configureDemoPeers()
        {
            // Tạo danh sách MAC giả lập để test cơ chế scheduler
            for (uint8_t group = 0; group < kGroupCount; ++group)
            {
                Group &groupRef = groups_[group];
                groupRef.count = kGroupSize;
                for (uint8_t index = 0; index < kGroupSize; ++index)
                {
                    PeerDevice &device = groupRef.peers[index];
                    device.channel = 1;                                     // Cố định ở kênh 1
                    device.encrypt = false;                                 // Không dùng LMK trong ví dụ
                    std::fill(std::begin(device.lmk), std::end(device.lmk), 0x00); // LMK rỗng

                    // Sinh MAC duy nhất dựa vào group/index để quan sát log dễ dàng
                    device.mac[0] = 0x24;
                    device.mac[1] = 0x6F;
                    device.mac[2] = 0x28;
                    device.mac[3] = static_cast<uint8_t>(0x80 + group);
                    device.mac[4] = index;
                    device.mac[5] = static_cast<uint8_t>(0x10 + group);
                }
            }
        }

        void setGroupPeers(uint8_t groupId, const PeerDevice *devices, size_t count)
        {
            // Bỏ qua nếu groupId không hợp lệ hoặc không có dữ liệu đầu vào
            if (groupId >= kGroupCount || devices == nullptr)
            {
                return;
            }

            Group &group = groups_[groupId];
            group.count = std::min<size_t>(count, kGroupSize);
            for (uint8_t idx = 0; idx < group.count; ++idx)
            {
                // Sao chép cấu hình peer vào nhóm tương ứng
                group.peers[idx] = devices[idx];
            }
        }

        void clearGroup(uint8_t groupId)
        {
            // Đặt lại số peer về 0 nếu groupId hợp lệ
            if (groupId >= kGroupCount)
            {
                return;
            }
            groups_[groupId].count = 0;
        }

        void begin()
        {
            // Bật scheduler, ghi nhận thời gian và kích hoạt nhóm đầu tiên
            initialized_ = true;
            lastSwitchMs_ = millis();
            activateGroup(0);
        }

        void update()
        {
            // Không làm gì nếu chưa gọi begin()
            if (!initialized_)
            {
                return;
            }

            unsigned long now = millis();
            // Kiểm tra chưa tới thời điểm chuyển slot thì thoát
            if (now - lastSwitchMs_ < slotDurationMs_)
            {
                return;
            }

            lastSwitchMs_ = now;
            uint8_t nextGroup = (activeGroup_ >= kGroupCount) ? 0 : static_cast<uint8_t>((activeGroup_ + 1) % kGroupCount);
            activateGroup(nextGroup);
        }

        uint8_t activeGroup() const
        {
            // Nếu chưa có nhóm nào thì trả về 0 làm mặc định
            return activeGroup_ < kGroupCount ? activeGroup_ : 0;
        }

        uint8_t activePeerCount() const
        {
            // Trả về số peer đang hoạt động ở nhóm hiện tại
            if (activeGroup_ >= kGroupCount)
            {
                return 0;
            }
            return groups_[activeGroup_].count;
        }

    private:
        struct Group
        {
            uint8_t count;                   // Số peer đã cấu hình trong nhóm
            PeerDevice peers[kGroupSize];    // Mảng lưu thông tin peer
        };

        Group groups_[kGroupCount]; // Bộ nhớ lưu cấu hình của tất cả các nhóm
        uint8_t activeGroup_;       // Nhóm đang được đăng ký với ESP-NOW
        uint32_t slotDurationMs_;   // Thời lượng mỗi slot được cấu hình
        unsigned long lastSwitchMs_; // Dấu thời gian chuyển slot gần nhất
        bool initialized_;          // Đảm bảo begin() được gọi trước khi update()

        void activateGroup(uint8_t groupId)
        {
            // Chỉ xử lý khi groupId hợp lệ và khác nhóm hiện tại
            if (groupId >= kGroupCount)
            {
                return;
            }
            if (groupId == activeGroup_)
            {
                return;
            }

            clearActivePeers();
            activeGroup_ = groupId;
            addPeersForGroup(groupId);
            logGroupChange(groupId);
        }

        void clearActivePeers()
        {
            // Nếu chưa có nhóm kích hoạt thì không cần xóa
            if (activeGroup_ >= kGroupCount)
            {
                return;
            }

            Group &group = groups_[activeGroup_];
            for (uint8_t index = 0; index < group.count; ++index)
            {
                const PeerDevice &peer = group.peers[index];
                esp_err_t err = esp_now_del_peer(peer.mac);
                if (err != ESP_OK && err != ESP_ERR_ESPNOW_NOT_FOUND)
                {
                    // Ghi log khi không xóa được peer khỏi bảng ESP-NOW
                    Serial.printf("[ESP-NOW] Failed to remove peer %u from group %u, err=%d\n", index, activeGroup_, err);
                }
            }
        }

        void addPeersForGroup(uint8_t groupId)
        {
            Group &group = groups_[groupId];
            for (uint8_t index = 0; index < group.count; ++index)
            {
                const PeerDevice &peer = group.peers[index];
                esp_now_peer_info_t info = {};
                memcpy(info.peer_addr, peer.mac, sizeof(peer.mac)); // Sao chép địa chỉ MAC vào cấu trúc ESP-NOW
                info.channel = peer.channel;                        // Kênh Wi-Fi đích
                info.encrypt = peer.encrypt;                        // Bật tắt mã hóa
                info.ifidx = WIFI_IF_STA;                           // Sử dụng giao diện STA
                if (peer.encrypt)
                {
                    memcpy(info.lmk, peer.lmk, sizeof(peer.lmk));  // Nạp LMK khi cần mã hóa
                }

                esp_err_t err = esp_now_add_peer(&info);
                if (err != ESP_OK)
                {
                    // Ghi log khi không add được peer vào danh sách đang hoạt động
                    Serial.printf("[ESP-NOW] Failed to add peer %u to group %u, err=%d\n", index, groupId, err);
                }
                else
                {
                    logPeer(peer, index);
                }
            }
        }

        void logGroupChange(uint8_t groupId) const
        {
            // Log tổng quan sau mỗi lần chuyển nhóm để dễ theo dõi
            Serial.printf("[ESP-NOW] Switched to group %u (%u peers)\n", groupId, groups_[groupId].count);
        }

        void logPeer(const PeerDevice &device, uint8_t index) const
        {
            // In chi tiết từng peer được nạp vào slot hiện tại
            Serial.printf("  → Peer %u MAC %02X:%02X:%02X:%02X:%02X:%02X%s\n",
                          index,
                          device.mac[0], device.mac[1], device.mac[2],
                          device.mac[3], device.mac[4], device.mac[5],
                          device.encrypt ? " [ENC]" : "");
        }
    };
}

#endif // HUB66S_ESPNOW_SCHEDULER_H