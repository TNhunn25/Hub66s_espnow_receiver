# Hub66s ESP-NOW Receiver

## Kiến trúc mạng
- **Coordinator (Hub)**: ESP32-S3 đóng vai trò trung tâm, khởi tạo ESP-NOW ở chế độ STA và đặt PMK mã hóa chung cho toàn bộ mạng.
- **Phân nhóm**: 100 nút thiết bị được chia thành 5 nhóm (G0–G4), mỗi nhóm 20 thiết bị. Mỗi thiết bị ngoại vi lưu `group_id` của mình trong NVS để Hub có thể ghép nhóm tương ứng.
- **Bảng peer động**: Hub chỉ đăng ký tối đa 20 peer cùng lúc (giới hạn ESP-NOW khi bật mã hóa). Trước mỗi slot hoạt động, Hub xóa toàn bộ peer của nhóm cũ và nạp peer của nhóm mới.
- **Lịch hoạt động**: các nhóm hoạt động luân phiên theo slot 1 giây. Chu kỳ khởi đầu ở nhóm G0 và quay vòng G4 → G0.

| Thứ tự | Nhóm | Thời gian | Hành động |
|--------|------|-----------|-----------|
| 1      | G0   | 0–1 s     | Hub nạp peer nhóm 0, trao đổi dữ liệu. |
| 2      | G1   | 1–2 s     | Hủy peer nhóm 0, nạp peer nhóm 1. |
| 3      | G2   | 2–3 s     | Tiếp tục vòng lặp. |
| 4      | G3   | 3–4 s     | … |
| 5      | G4   | 4–5 s     | … |

## Thành phần chính
- `espnow_handler.h`: khởi tạo ESP-NOW, thiết lập PMK và callback gửi/nhận.
- `espnow_scheduler.h`: quản lý danh sách peer theo nhóm, thực hiện logic luân phiên slot.
- `receiver.ino`: vòng lặp chính cập nhật scheduler, xử lý license, LED và watchdog.

> **Lưu ý:** `configureDemoPeers()` chỉ cung cấp danh sách MAC mẫu để minh họa cơ chế. Khi triển khai thực tế, hãy gọi `setGroupPeers()` với MAC và khóa LMK thật của từng thiết bị.