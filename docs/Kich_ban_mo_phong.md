KỊCH BẢN MÔ PHỎNG ĐỀ XUẤT (DSDV + GRADIENT – nRF54L15 – BLE)

1.  Mục tiêu và phạm vi thử nghiệm:

1.1 Mục tiêu chung

Đánh giá và so sánh hiệu năng định tuyến của hai thuật toán:

- DSDV (Proactive routing)

- Gradient-based routing

Trong bối cảnh:

- Mạng WSN ad-hoc

- Nền tảng nRF54L15 – BLE Mesh

- Điều kiện triển khai thực tế (real deployment)

> 1.2 Các khía cạnh đánh giá

- Hiệu suất truyền dữ liệu

- Chi phí điều khiển mạng

- Độ ổn định và khả năng thích nghi topo

- Khả năng mở rộng mạng

- Tiêu thụ năng lượng (phù hợp WSN)

1.3 Phạm vi

> Số lượng node: 20 – 30 – 46

- Môi trường: indoor, có vật cản

> So sánh song song:

- Mô phỏng (quy mô lớn)

- Thực nghiệm (≤ 50 node)

2.  Các chỉ số đánh giá (Metrics):

> Hiệu suất truyền dữ liệu:

- Packet Delivery Ratio (PDR)

- End-to-End Delay

- Hop Count trung bình

> Chi phí mạng:

- Control Overhead (số gói control / tổng gói).

> Độ ổn định và thích nghi:

- Thời gian hội tụ ban đầu.

- Recovery time sau sự cố.

- Route change frequency.

- Tần suất thay đổi tuyến/bảng định tuyến

  - <span class="mark">Đo trong 1 khoảng thời gian</span>

  - <span class="mark">Thay đổi node, kích thích (số lần kích thích – số lần thay đổi) …</span>

> Năng lượng (đề xuất):

- Số lần TX / RX.

- Thời gian CPU active.

- Năng lượng tiêu thụ / gói tin thành công.

2.1 Hiệu suất truyền dữ liệu

- Packet Delivery Ratio (PDR)

- End-to-End Delay

- Hop Count trung bình

1.  Packet Delivery Ratio (PDR)

> Packet Delivery Ratio (PDR) là tỷ lệ giữa số gói dữ liệu nhận thành công tại đích và số gói dữ liệu được gửi từ nguồn trong một khoảng thời gian xác định.
>
> <img src="./images/media/image1.png" style="width:1.15725in;height:0.50249in" />
>
> Trong đó:

- N<sub>sent</sub>​ : tổng số gói **DATA** được phát đi.

- N<sub>received</sub>​: tổng số gói **DATA** được nhận thành công tại đích.

> Quy trình đo PDR (áp dụng cho cả 2 phương án):

- **Regular nodes** → gửi DATA → **Sink node**

- Sink chỉ nhận, không phản hồi

> **Bước 1: Chờ mạng ổn định.**

- Đợi mạng hội tụ:

  - DSDV: bảng định tuyến ổn định

  - Gradient: gradient ổn định (sink = 0, các node khác không đổi)

- Sẽ không tính gói trong giai đoạn này.

> **Bước 2: Bắt đầu đo.**

- Nhấn nút START:

  - Dùng lệnh comment để chọn node gửi gói tin tới địa chỉ mong muốn.

- Lưu thời điểm: Các node bắt đầu gửi gói DATA định kỳ (mỗi 𝑇 giây). Theo tài liệu của Nordic là trên 0.5ms/gói tin.

  - Trong WSN / BLE Mesh: Chu kỳ gửi T phải lớn hơn thời gian xử lý gói tin ít nhất 1–2 bậc độ lớn (10–100×)

| **Thành phần**     | **Thời gian ước lượng** |
|--------------------|-------------------------|
| BLE packet TX      | ~0.5–1 ms               |
| Relay / Mesh delay | 1–5 ms                  |
| CPU xử lý          | 0.5–2 ms                |

- Trường hợp 1: Đo PDR / hiệu suất ổn định (KHUYẾN NGHỊ): T=1s

- Trường hợp 2: Test tải trung bình: T = 500 ms.

- Trường hợp 3: Stress test: T = 100–200 ms.

> **Bước 3: Kết thúc đo**

- Nhấn nút **STOP**

- Lưu thời điểm t kết thúc.

> **Bước 4: Thu thập thống kê**

- Mỗi node duy trì bộ đếm:

  - data_sent

  - data_received.

2.  End-to-End Delay (độ trễ):

> End-to-End Delay là thời gian từ lúc gói DATA được tạo tại node nguồn cho đến khi gói đó được nhận thành công tại sink node**.**
>
> <img src="./images/media/image2.png" style="width:2.37533in;height:0.50007in" />
>
> Trong đó:

- t<sub>send</sub>​: thời điểm node nguồn gửi DATA.

- t<sub>receive</sub>​: thời điểm sink nhận DATA tương ứng.

> **Quy trình đo End-to-End Delay:**

- Chỉ đo delay SAU KHI mạng đã ổn định.

> **Bước 1: Chờ mạng ổn định**

- Đợi mạng hội tụ:

  - DSDV: bảng định tuyến ổn định

  - Gradient: gradient ổn định (sink = 0, các node khác không đổi)

- Sẽ không tính gói trong giai đoạn này.

> **Bước 2: Bấm START**

- <span class="mark">Đợi ổn định (5p) rồi bắt đầu truyền</span>

- <span class="mark">Package phải đủ dữ lệu</span>

> **Bước 3: Bắt đầu ghi delay**

- Kết quả mong muốn:

> Average End-to-End Delay:

- D<sub>Average.</sub>

> <img src="./images/media/image3.png" style="width:1.97623in;height:0.47658in" />

- D<sub>min</sub>​.

- D<sub>max</sub>.

3.  Hop Count trung bình

> Hop Count là số lượng node trung gian mà một gói DATA phải đi qua để đến được sink node.
>
> Nếu:

- **Node** → Sink trực tiếp → Hop = 1

- Qua 2 node trung gian → Hop = 3

  2.  Chi phí mạng:

> • Control Overhead (số gói control / tổng gói).

1.  Overhead gói tin:

> **Overhead gói tin (Control Overhead)** là tỷ lệ hoặc số lượng các gói tin không mang dữ liệu ứng dụng**,** nhưng cần thiết để thiết lập, duy trì và cập nhật định tuyến trong mạng.
>
> <img src="./images/media/image4.png" style="width:2.56286in;height:0.53132in" />
>
> Trong đó:

- N<sub>control</sub>​: tổng số gói control được gửi trong thời gian đo

- N<sub>data</sub>​: tổng số gói data được gửi trong thời gian đo

> Ý nghĩa đánh giá

- Giá trị thấp → thuật toán tiết kiệm tài nguyên, phù hợp WSN

- Giá trị cao → chi phí điều khiển lớn, ảnh hưởng đến năng lượng và khả năng mở rộng

> Gói CONTROL (tính overhead):

- Gradient

  - Bản tin beacon (dùng để cập nhật gradient và các giá trị)

  - Heartbeat: duy trì định tuyến ngược.

- DSDV:

  - Routing table update

  - Periodic update

  - Triggered update

> 2.3 Độ ổn định và thích nghi
>
> • Thời gian hội tụ.
>
> • Recovery time sau sự cố.
>
> • Route change frequency.

1.  Thời gian hội tụ:

> Thời gian hội tụ là khoảng thời gian từ khi mạng được khởi tạo đến khi trạng thái định tuyến của mạng trở nên ổn định, cho phép truyền dữ liệu một cách tin cậy.
>
> <img src="./images/media/image5.png" style="width:1.90652in;height:0.33338in" />
>
> Trong đó:

- t<sub>start</sub>​: thời điểm khởi tạo mạng

- t<sub>stable</sub>​: thời điểm mạng đạt trạng thái ổn định

> Tiêu chí xác định ổn định:
>
> Mạng được coi là ổn định khi:

- Không còn cập nhật định tuyến mới

- Next-hop của các node không thay đổi trong khoảng T<sub>stable</sub>

- Truyền dữ liệu test thành công với PDR cao

  1.  Recovery Time sau sự cố (Failure Recovery Time):

> Recovery time là khoảng thời gian từ khi xảy ra sự cố (node/link failure) đến khi mạng khôi phục được khả năng truyền dữ liệu ổn định.
>
> <img src="./images/media/image6.png" style="width:1.80233in;height:0.36463in" />
>
> Trong đó:

- t<sub>fail</sub>​: thời điểm xảy ra sự cố.

- t<sub>recover</sub>​: thời điểm PDR hoặc tuyến ổn định trở lại.

> Mạng được coi là đã phục hồi khi:

- Dữ liệu lại được truyền liên tục về sink

- Không còn route change liên tục

- PDR đạt lại ≥ ngưỡng định trước (ngưỡng 95%)

  1.  Route Change Frequency:

> Route Change Frequency là số lần thay đổi tuyến (next-hop) của các node trong một đơn vị thời gian, phản ánh mức độ ổn định của định tuyến.
>
> <img src="./images/media/image7.png" style="width:1.76728in;height:0.60537in" />
>
> Trong đó:

- N<sub>route_change</sub>​: tổng số lần thay đổi next-hop

- T<sub>measure</sub>​: thời gian đo

3.  Kịch bản thử nghiệm 1: Mạng tĩnh – đánh giá hiệu suất nền

3.1 Mục đích:

- Đánh giá hiệu suất tối ưu của từng thuật toán trong điều kiện lý tưởng

- Làm mốc so sánh cho các kịch bản

> 3.2 Thiết lập:

- Topology:

  - Tree cân bằng

  - Grid đều

- Sink cố định

- Traffic: mỗi node gửi 1 gói / T giây

> 3.3 Nội dung đo:

- PDR

- Delay

- Hop count

- Overhead

3.4 Thay đổi quy mô mạng:

- So sánh với:

- 20 node

- 30 node

- 46 node

3.5 Thiết lập

- Chạy liên tục 6–12 giờ

- Gửi dữ liệu định kỳ

4.  Tổng hợp và so sánh kết quả:

> 4.1 So sánh định lượng:

- Bảng tổng hợp metric theo từng kịch bản

  2.  Phân tích định tính

- Điểm mạnh – điểm yếu của từng thuật toán.

Gradient: - đánh nhãn các gói tin gói control(Các gói tin hello, routing, ACK), gói DATA

- PDR: BS Nhận dữ liệu từ tất cả các node gửi đến: loại trường dữ liệu, đếm số trường DATA (sẽ đo được PDR, trước khi kết thúc phải hỏi node đã gửi bao nhiêu bản tin để lấy tỷ số truyền và nhận, trên thằng phát cũng phải đếm số gói tin đã gửi ra).

- Packet Overhead: Gói Control: đếm trong khoảng thời gian có bao nhiêu gói tin được phát + thêm các gói tin hello đã tính trên lý thuyết + thêm ACK, chia cho thời gian

- Packet nhận được phải có trường hopcount: có thể tính bằng TTL

- Route chang fraquency: đo trong một mạng tĩnh xem tần suất thay đổi bảng định tuyến là bao nhiêu, sau đó cưỡng bức thay đổi thì đo xem có thay đổi đúng hay không

- Thêm một kịch bản động

- Chốt khoảng thời gian để chạy

- Thử với tần số chạy khác nhau

- Thu được số liệu tách ra để vẽ đồ thị

- Add seq number vào 1 trường trong bản tin, nó sẽ +1 mỗi khi gửi một bản tin, trên sinknode sẽ so sánh giá trị giữa seq number mới và đã lưu trước nếu lớn hơn thì lấy giá trị lớn hơn (tức là giá trị mới) sau đó thì chỉ cần lấy số seq number đó thì sẽ là gói tin nhận được trên sinknode. Sau khi nhận được gói cuối thì gửi ACK kèm với số seq đã lưu cuối cùng về node nhận được ACK biết để ngừng gửi
