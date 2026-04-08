# Hướng dẫn chạy Stress Test - Hệ thống Gradient Mesh BLE

Tài liệu này hướng dẫn cách thiết lập, cấu hình và thực hiện bài kiểm tra độ tin cậy (Stress Test) cho hệ thống mạng Mesh sử dụng giao thức định tuyến Gradient.

---

## 1. Tổng quan hệ thống
Hệ thống bao gồm ba thành phần chính:
- **Sink Node (Gateway):** Đầu mối tập trung dữ liệu (Gradient = 0). Kết nối trực tiếp với máy tính qua UART.
- **Sensor Nodes:** Các nút cảm biến thu thập dữ liệu và gửi về Sink Node thông qua định tuyến Gradient.
- **Gateway Service (Python):** Dịch vụ chạy trên máy tính để thu thập log, quản lý Topology và lưu trữ kết quả kiểm tra.

---

## 2. Chuẩn bị
### Phần cứng
- Board nRF54 hoặc nRF52 series (DK).
- Cáp micro-USB hoặc USB-C để kết nối UART.

### Phần mềm
- Python 3.8+
- Các thư viện Python cần thiết:
  ```bash
  pip install pyserial fastapi uvicorn networkx
  ```

---

## 3. Cấu hình Firmware
Trước khi biên dịch và nạp code, bạn có thể điều chỉnh các tham số kiểm tra trong file `src/model_handler.c`:

- `TEST_DURATION_MINUTES`: Thời gian chạy bài test tự động (mặc định 45 phút).
- `SENSOR_SEND_INTERVAL_MS`: Tần suất gửi tin của các Sensor Node (ví dụ: 5000ms). **Các e có thể tăng tần suất lên 30 phút hoặc 1 tiếng tùy vào kết quả sau khi đã test xong, a nghĩ nên đặt 30 phút trước sau đó phân tích kết quả rồi mới set tiếp**
- **Lưu ý**: Khi Flash code và provision cho các node thì sink node phải có địa chỉ là 0x0002.
- Nhớ đảm bảo cổng UART không bị chiếm dụng bởi ứng dụng nào khác như nrf connect for desktop chẳng hạn.
---

## 4. Quy trình thực hiện Stress Test

### Bước 1: Khởi động Gateway PC
Kết nối Sink Node vào máy tính, kiểm tra cổng COM (mặc định trong `Gateway.py` là `COM32`) hoặc các em có thể thay đổi ở dòng **30** trong **Gateway.py** theo cổng COM khi cắm uart trên máy tính và chạy lệnh:
```bash
python Gateway.py
```
Sau đó, truy cập index.html để xem giao diện Topology thời gian thực.

### Bước 2: Kích hoạt Test (Broadcast Mode)
Bạn có thể bắt đầu bài test cho **toàn bộ mạng** bằng cách:
1. **Nút bấm vật lý:** Nhấn **Nút 3 (là nút S2)** trên board **Sink Node**.

**Dấu hiệu nhận biết:**
- Đèn LED trên các Node sẽ nháy ở chế độ **Attention** để báo hiệu đang trong phiên test.
- Các Sensor sẽ bắt đầu gửi dữ liệu tự động về Sink sau một khoảng thời gian trễ ngẫu nhiên (Staggered Start).

### Bước 3: Theo dõi và kết thúc
- Theo dõi các gói tin và biểu đồ mạng trên Dashboard.
- Khi hết thời gian `TEST_DURATION_MINUTES`, Sink Node sẽ tự động gửi lệnh **STOP** và yêu cầu các Sensor báo cáo thống kê (PDR, RSSI...).
- Nếu muốn dừng thủ công, nhấn lại **Nút 3** trên Sink Node.

### Bước 4: Lấy kết quả
- sau khi chạy hãy đợi 5 - 10 phút để dữ liệu được đẩy hết quả uart và lưu vào log
- file Final Log sẽ tự động được sinh ra tại vị trí **wsn_backup**
- Tên file định dạng: `GRADIENT_FINAL_STRESS_{SESSION_ID}_{TIMESTAMP}.csv`
- Hãy lưu lại file log FINAL sau mỗi lần chạy xong hoặc có thể gửi qua cho a luôn để a phân tích trước.
---



