# Proposal: Holistic Network State Prediction & Optimization

This plan outlines the architectural shift from the current **per-link cost prediction** (using LightGBM) to a **holistic network-aware AI system**.

---

## 🌟 Tóm tắt dễ hiểu - "Google Maps cho mạng Mesh"

Bạn có thể hình dung thuật toán AI (GNN) này hoạt động trên Raspberry Pi như sau:

1. **Vẽ lại "bức tranh toàn cảnh"**: Máy chủ thu thập dữ liệu để vẽ ra một "bản sao kỹ thuật số" của mạng. Nó biết rõ trạm nào nối với trạm nào, pin còn bao nhiêu, sóng mạnh hay yếu và có hay rớt gói tin không.
2. **"Hỏi thăm láng giềng" (Lan truyền thông tin)**: Đây là linh hồn của GNN. Các trạm ảo trong máy chủ sẽ "hỏi thăm" lẫn nhau. Nếu trạm cha sắp quá tải, trạm con sẽ "cảm nhận" được ngay và chủ động tìm đường khác trước khi sự cố thực sự xảy ra.
3. **Đưa ra "lời tiên tri"**: AI dự báo trước điểm nào sắp kẹt xe (nghẽn mạch) và tuyến đường nào là an toàn nhất cho cả một hành trình dài, chứ không chỉ nhìn vào một chặng ngắn trước mắt.
4. **"Bẻ lái" dòng dữ liệu**: Khi đã thấy điểm đen, máy chủ sẽ ra lệnh cho các trạm bên dưới đổi hướng đi. Dòng dữ liệu sẽ tự động né chỗ tắc để về đích an toàn.

**Tóm lại**: Nó giống như **Google Maps** – luôn nhìn trước các ngã tư phía trước để chỉ đường vòng cho bạn trước khi bạn bị kẹt cứng trong đám đông!

---

## Technical Analysis of Current State

Currently, the AI (LightGBM):
- Predicts `Routing_Cost` for individual links based on local features (`RSSI`, `Drop_Rate`, `Pin`).
- Operates within a `Gateway_main.py` loop where the global routing is solved by Dijkstra based on these local costs.
- **Limitation:** It lacks "global context." For example, it doesn't know if a low-cost link points to a node that is a critical bottleneck for the rest of the network.

## Proposed Components for Holistic AI

## Deep Dive: Phase 1 - Graph Neural Networks (GNNs)

Trong dự án của bạn, GNN sẽ đóng vai trò như một "bộ não" có cái nhìn toàn cảnh về mạng, thay vì chỉ nhìn vào từng liên kết riêng lẻ.

### 1. Cách biểu diễn mạng dưới dạng Đồ thị (Graph Representation)
- **Node (Nút)**: Mỗi cảm biến nRF54. Đặc trưng của mỗi node (Node Features) bao gồm: `Drop_Rate`, `Fwd_Count`, `Pin (%)`, `Grad`.
- **Edge (Cạnh)**: Các kết nối vật lý giữa các node. Đặc trưng của cạnh (Edge Features) bao gồm: `RSSI`, `Link_UP (s)`.
- **Target (Mục tiêu)**: Dự đoán xác suất tắc nghẽn (Congestion Probability) hoặc độ tin cậy của đường truyền trong 5-10 phút tới.

### 2. Cơ chế "Truyền tin" (Message Passing) - Trái tim của GNN
Đây là điểm khác biệt lớn nhất so với LightGBM:
- **Bước 1 (Aggregation)**: Mỗi node sẽ thu thập trạng thái từ các node láng giềng xung quanh nó (ví dụ: Node A biết được node cha B đang bị quá tải 90% bộ đệm).
- **Bước 2 (Update)**: Node A cập nhật trạng thái của chính nó dựa trên thông tin từ node B. 
- **Kết quả**: Sau 2-3 vòng truyền tin (layers), một node ở xa có thể "cảm nhận" được tình trạng nghẽn mạch ngay tại Gateway và chủ động tìm đường vòng sớm hơn.

### 3. Tác dụng cụ thể với dự án BLE Mesh
- **Phát hiện nút thắt cổ chai (Bottleneck)**: GNN sẽ nhận diện được các node "huyết mạch" mà nhiều luồng dữ liệu đi qua để điều phối traffic sang các nhánh khác trước khi node đó bị treo.
- **Dự báo năng lượng hệ thống**: Thay vì chỉ biết pin của chính mình, GNN hiểu được sự cân bằng pin của cả cụm (cluster) để đưa ra quyết định routing giúp kéo dài tuổi thọ toàn mạng.
- **Tối ưu hóa tham số SDN**: GNN có thể đưa ra các tham số cấu hình mạng (như `topo_req` interval) biến thiên theo mật độ và độ ổn định của đồ thị hiện tại.

## Cơ chế thu thập dữ liệu (Data Flow) - Giải đáp lo ngại về băng thông

Một ưu điểm cực kỳ quan trọng là: **Bạn KHÔNG cần thay đổi Firmware hay gửi thêm bất kỳ tham số mới nào từ các node lên.**

### 1. Dữ liệu gốc đã có sẵn
Hiện tại, bản tin `$[TOPO]` từ các node nRF54 gửi về Gateway (UART) đã chứa đầy đủ "nguyên liệu" cho GNN:
- **Header**: Đã có `Origin`, `Grad`, `Drops`, `FwdRate`, `Uptime`, `TotalSent` (dùng để tính Pin).
- **Neighbor List**: Đã có `addr`, `rssi`, `grad`, `nb_uptime` của tất cả láng giềng.

### 2. GNN chạy tại Gateway (Tập trung)
Cơ chế "Truyền tin" (Message Passing) mà tôi nhắc đến ở Phase 1 thực chất là **"Ảo"**: 
- Nó diễn ra hoàn toàn trong bộ nhớ của Gateway (Raspberry Pi/PC). 
- Thay vì các node thật phải truyền tin cho nhau qua sóng Mesh (gây tốn băng thông), GNN sẽ mô phỏng quá trình này dựa trên bản đồ mạng mà nó đã nhận được từ UART.
- **Lợi ích**: Bạn có sức mạnh của một thuật toán AI "biết tuốt" toàn mạng mà không làm tốn thêm dù chỉ 1 byte băng thông của mạng nRF54.

### 3. Quy trình khép kín
1. **Node** gửi TOPOLOGY định kỳ (như hiện tại).
2. **Gateway** nhận dữ liệu -> Xây dựng đồ thị trong Python.
3. **GNN Model** (chạy trên Gateway) "nhìn" vào đồ thị -> Dự đoán node nào sắp nghẽn, link nào sắp yếu.
4. **Gateway** gửi lệnh `backprop` (như hiện tại) để điều hướng các node trước khi vấn đề xảy ra.

## Luồng tính toán chi tiết của GNN (Inference Workflow)

Sau khi Gateway nhận được dữ liệu UART và xây dựng đồ thị, quá trình tính toán "hiểu mạng" sẽ diễn ra qua 4 bước chính:

### Bước 1: Chuyển đổi đồ thị thành Ma trận (Tensor Conversion)
Python sẽ chuyển `Master_Graph` thành 2 ma trận số học:
- **Ma trận kề (Adjacency Matrix - $A$)**: Biểu diễn các kết nối Mesh.
- **Ma trận đặc trưng (Feature Matrix - $X$)**: Chứa toàn bộ RSSI, Drops, Pin của tất cả các node.

### Bước 2: Quá trình truyền tin qua các lớp (Layer Processing)
Mô hình GNN (ví dụ: `GraphSAGE` hoặc `GAT`) thực hiện phép toán:
$h_{v}^{(k)} = \sigma(W \cdot \text{Aggregate}(h_{v}^{(k-1)}, \{h_{u}^{(k-1)} : u \in N(v)\}))$
- **Nghĩa là**: Ở mỗi lớp, máy tính sẽ nhân các trọng số ($W$) with thông tin của chính nó and cộng với trung bình có trọng số của các nút láng giềng. Sau 2-3 lớp, mỗi node sẽ mang trong mình "vong hồn" dữ liệu của toàn bộ khu vực xung quanh nó.

### Bước 3: Đưa ra dự báo (Output Head)
Tầng cuối cùng của GNN (thường là một tầng Regressor) sẽ xuất ra các chỉ số:
- **Congestion Score ($C_v$)**: Xác suất node $v$ sẽ bị nghẽn trong 5 phút tới.
- **Path Reliability ($R_v$)**: Độ tin cậy của cả lộ trình từ node $v$ về Sink (không chỉ là 1 hop).

### Bước 4: Ra quyết định (Decision & Action)
Gateway sẽ so sánh:
- Nếu $C_{\text{Parent Hiện tại}}$ cao hơn $C_{\text{Parent Tiềm năng}}$, hệ thống sẽ tự động kích hoạt lệnh `backprop` để đổi hướng.

## Deep Dive: Phase 2 - Digital Twin & DRL

Move from "calculating cost" to "learning a routing policy."

- **Digital Twin:** Create a simulation (using `NS-3` or a custom lightweight Python simulator) that uses your historical `topology_log.csv` data to model real-world node behavior.
- **DRL Agent:** An agent (e.g., using `PPO` or `DQN`) learns to assign `Parent` IDs to nodes to maximize a global reward function:
    - `Reward = (PDR_Overall * W1) + (Avg_Battery_Life * W2) - (Avg_Latency * W3)`.
- **Benefit:** The AI optimizes for the *entire network's health* rather than just minimizing a sum of local costs.

### 3. Spatio-Temporal Prediction (GNN + LSTM)
The network state changes over time. Predicting *future* congestion is key.

- **Architecture:** Feed GNN-extracted features into an LSTM or GRU to predict the state of the graph at $T+10$ minutes.
- **Use Case:** Proactively redirect traffic *before* a node dies or a buffer overflows.

## Transition Strategy

### Phase 1: Feature Engineering (Short Term)
Keep LightGBM but add "Graph-aware" features to the dataset:
- **Centrality:** How many shortest paths go through this node?
- **Neighborhood Load:** Total traffic of all neighbors.
- **Distance to Sink:** Beyond just `Grad`, use physical or hop-based distance metrics.

### Phase 2: GNN Integration (Medium Term)
Implement a GNN-based `Global_State_Predictor.py`:
- Use current `topology_log.csv` where each "Origin" report is treated as a snapshot of a graph.
- Train the GNN to predict `Network_PDR` based on the current `Master_Graph` configuration.

### Phase 3: DRL Control (Long Term)
The GNN/ML model provides the "State," and a DRL agent provides the "Action" (`backprop` commands).

## Verification Plan

### Automated Simulation
- Use the **Digital Twin** to compare:
    - Base Heuristic Routing
    - LightGBM Link-Local Routing (Current)
    - GNN-aware Routing (Proposed)
- Metric: Total network throughput and standard deviation of node energy levels (lower is better for longevity).

### Lab Deployment
- Deploy to the `nRF54` nodes and observe the `backprop` frequency.
- Holistic AI should result in fewer but more strategic route changes.
