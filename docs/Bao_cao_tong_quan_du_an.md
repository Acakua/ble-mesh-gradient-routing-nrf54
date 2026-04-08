# Báo cáo Chi tiết Tổng quan Dự án: Phân định Định tuyến Gradient BLE Mesh với Trí tuệ Nhân tạo (AI/SDN)

## 1. Giới thiệu Tổng quan
Dự án **GRADIENT_SRV** là một hệ thống mạng vô tuyến cảm biến (Wireless Sensor Network - WSN) dựa trên công nghệ **Bluetooth Mesh** (sử dụng nRF Connect SDK / Zephyr RTOS). Cốt lõi của dự án là giao thức **Gradient Routing** kết hợp với kiến trúc điều khiển mạng định nghĩa bằng phần mềm (**SDN** - Software-Defined Networking) và **Trí tuệ Nhân tạo (AI)** để tối ưu hóa đường truyền dữ liệu (Routing) theo thời gian thực.

Dự án sử dụng Gateway là Raspberry Pi 3B để điều khiển và thu thập dữ liệu từ các node cảm biến nRF54l15. Các node cảm biến nRF54l15 được kết nối với nhau thông qua Bluetooth Mesh và được điều khiển bởi Gateway.

Mục tiêu của hệ thống là tự động hóa và tối ưu hóa việc định tuyến mạng phân tán, giải quyết các vấn đề như nút thắt cổ chai (bottleneck), thất thoát gói tin (packet drop), và hao hụt năng lượng (pin) không đồng đều trên các thiết bị nRF. Đồng thời giảm thiểu chi phí thiết kế và kiểm chứng khả năng thực tế của việc chạy AI trên các thiết bị biên

---

## 2. Kiến trúc Hệ thống

Hệ thống được chia làm hai phân hệ chính: **Phân hệ Nhúng (Firmware)** chạy trên các node cảm biến và **Phân hệ Gateway (Phần mềm)** chạy trên máy chủ điều khiển trung tâm (Raspberry Pi/PC).

### 2.1. Phân hệ Nhúng (nRF Firmware bằng C)
Các thiết bị trong mạng chạy Firmware phát triển từ mã nguồn C (Zephyr RTOS), bao gồm các thành phần chính nằm trong thư mục `src/`:
- **Gradient Routing Manager (`gradient_srv.c`, `gradient_work.c`)**: Quản lý địa chỉ Gradient của mỗi node định hình khoảng cách logic tới Sink/Gateway.
- **Neighbor & Topology Management (`neighbor_table.c`, `heartbeat.c`)**: Liên tục theo dõi trạng thái các node láng giềng. Các thông số được ghi nhận bao gồm RSSI, độ ổn định link (Link Uptime), và Gradient.
- **Data & Traffic Monitoring (`data_forward.c`, `packet_stats.c`)**: Theo dõi số lượng gói tin đi qua (Forward Count) và số lần thất thoát/rớt gói tin (Drop Rate), qua đó đánh giá độ "nghẽn" của từng node.
- **Reverse Routing / Backprop (`reverse_routing.c`, `model_handler.c`)**: Cung cấp cơ chế nhận lệnh điều khiển cưỡng chế định tuyến từ Gateway xuống để thay đổi "Parent" (láng giềng trạm kế tiếp) nhằm né tránh vùng nghẽn.
- **Shell Commands (`shell_commands.c`)**: Hỗ trợ giao tiếp CLI (như cấu hình, cấp lệnh, gửi tin nhắn kiểm tra `chat msg`, `chat private`).

### 2.2. Phân hệ Trung tâm điều khiển Gateway (Python)
Chương trình `Gateway_main.py` đóng vai trò là "Bộ não" trung tâm, thực hiện các thao tác:
- **Thu thập dữ liệu Telemetry**: Đọc luồng dữ liệu định kỳ từ mạng Mesh qua cổng Serial UART. Bóc tách các bản tin `$[TOPO]` chứa thông tin về: Gradient, Nút cha (Parent), Danh sách láng giềng với RSSI, Tỷ lệ Drop gói tin và Pin ước lượng.
- **Trực quan hóa đồ thị (Real-time Dashboard)**: Cung cấp Web Server qua FastAPI kết hợp Websocket. Biểu diễn mạng Mesh dưới dạng đồ thị tương tác (Vis.js), với các độ trễ, cảnh báo quá tải được cập nhật trực tiếp cho người dùng.
- **Ghi nhận & Xử lý Stress Test**: Hỗ trợ thu thập dữ liệu sâu (Log CSV) để đánh giá PDR (Packet Delivery Ratio), Latency (Độ trễ) và Số lượng gói tin Heartbeat/Beacon, làm kho ngữ liệu phục vụ huấn luyện máy học.

---

## 3. Cơ chế Định tuyến thông minh (Smart Routing & AI/SDN)

Thay vì để các node tự phân mảnh và mò mẫm đường đi (như giao thức Mesh truyền thống), dự án đưa ra cơ chế quản lý lai (Hybrid) sáng tạo:

### 3.1. Tính toán Chi phí Định tuyến (Routing Cost) theo Link-Local (Mô hình LightGBM)
Gateway phân tích dữ liệu các láng giềng của mỗi node và dự đoán một "Chi phí đường truyền" (Routing Cost) bằng mô hình máy học **LightGBM**. Các feature (đặc trưng) để tiên đoán bao gồm: `Grad`, `RSSI`, `Link_UP (s)`, `Drop_Count`, `Fwd_Count`, `Drop_Rate (%)`, `Pin (%)` và mức độ phân tải `Load_Per_Neighbor`. 
Nếu không có mô hình AI, hệ thống tự động fallback về công thức tính toán tĩnh (Heuristic) truyền thống (dựa theo quy luật kết hợp phạt theo mức sóng, pin và trễ).

### 3.2. Điều khiển SDN tập trung & Thuật toán Dijkstra
Sau khi có độ suy hao của mọi liên kết, **Master Graph** sẽ được cập nhật. Thuật toán **Dijkstra** chạy trên đồ thị ảo (Virtual Graph) tại Gateway sẽ tìm ra tuyến đường (Next-hop / Parent) tốt nhất cho mọi node dẫn về Sink. 

### 3.3. Cơ chế kích hoạt Hybrid Push (Backpropagation)
- Cập nhật Delta: Hệ thống chỉ trích xuất ra những node đang đi **sai** luồng đường thiết kế tối ưu mới (AI_Parent khác Actual_Parent).
- Lệnh Điều phối chặn trước: Nếu số lượng sai lệch ít ($K \\leq 13$), Gateway phát lệnh Unicast (`mesh backprop`). Nếu thay đổi di chuyển theo cụm lớn, Gateway sẽ nhồi tất cả lệnh vào một gói Broadcast (`backprop_broadcast`) để toàn mạng tự đọc lệnh và diễn tiến đổi trạm ngay lập tức (Pending Commit / 2PC - Two Phase Commit).

---

## 4. Kết luận
Dự án **GRADIENT_SRV** thể hiện một giải pháp đột phá khi mang sự linh hoạt và phức tạp của AI (LightGBM/GNN)/SDN đặt lên vai Gateway (Resource-rich), trong khi giải phóng nRF Node (Resource-constrained) khỏi việc phải tự phán đoán đường truyền mù mờ. Bằng việc bóc tách Telemetry và tối ưu hóa tuyến đường tập trung, đây là một hệ thống IoT/WSN tối tân, tự phục hồi, chống nghẽn và tiết kiệm năng lượng toàn cục xuất sắc.
