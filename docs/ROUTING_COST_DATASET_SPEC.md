# Comprehensive Dataset Specification: Routing Cost Prediction for WSN

This document provides a deep technical dive into the parameters, mechanisms, and logic used to collect, process, and train the LightGBM model for calculating `Routing_Cost` in our Software Defined Network (SDN) over Bluetooth Mesh.

---

## 1. Document Objective
The goal is to maintain a high-performance Gradient Routing system where path selection is optimized by AI. This document serves as the single source of truth for:
- How firmware metrics are defined and sampled.
- How raw data is transformed into "features" for machine learning.
- How the Ground Truth (labels) are calculated for supervised training.

---

## 2. Parameter Definitions (Features)

The model utilizes **10 Features** to make a prediction. These are split into Raw Metrics (from nodes) and Engineered Features (at Gateway).

### A. Raw Features (Firmware Layer)
Captured periodically by the `packet_stats.c` and `gradient_srv.c` modules on each sensor node.

| Feature | Field Name | Type | Description & Calculation Logic |
|:---|:---|:---|:---|
| **Gradient** | `Grad` | `uint8` | **Hop Distance:** The node's current distance to the Sink. `0` = Sink. Lower is usually better, but AI might prefer a slightly longer path if it's more stable. |
| **Signal Strength** | `RSSI` | `int8` | **Link Quality:** RSSI captured during the most recent message from a neighbor. Threshold for reliable communication is typically **>-70dBm**. |
| **Uptime** | `Link_UP(s)` | `uint32` | **Reliability Window:** Seconds since the neighbor was first discovered. Long-running links are prioritized to avoid "flaky" new neighbors. |
| **Remaining Energy** | `Pin(%)` | `float` | **Survival Metric:** Estimated using an energy model: `100 - (Uptime * Idle_Power + TX_Count * TX_Power)`. Crucial for load balancing away from dying nodes. |
| **Dropped Packets** | `Drop_Count` | `uint16` | **Congestion Indicator:** Sum of **Soft Drops** (App buffer overflows) and **MAC Drops** (Hardware collisions/interference). |
| **Throughput** | `Fwd_Count` | `uint16` | **Load Indicator:** Total number of successful MAC transmissions (Local + Relayed) in the current 30s window. |

### B. Engineered Features (Gateway Transformation)
These features transform linear raw data into non-linear patterns that help the LightGBM model learn complex relationships.

| Feature | Calculation Formula | Technical Motivation |
|:---|:---|:---|
| **Drop Rate (%)** | `(Drops / (Drops + Fwd)) * 100` | Normalized reliability. Percentage is more descriptive than a raw counter for node health, regardless of traffic volume. |
| **Neighbor Count** | `count(neighbors)` | **Density Factor:** High neighbor count suggests high potential for radio interference and collisions. |
| **Link Stability** | `log1p(Link_UP)` | **Log-Scaling:** Uptime varies from 1s to hours. Log-scaling ensures that the difference between 10s and 60s is weighted correctly compared to 3600s and 3650s. |
| **Traffic Load** | `Fwd_Count / (NB_Count + 1)` | **Congestion Level:** Distinguishes between a node that is busy because it has many neighbors vs. a node that is congested with high-frequency traffic. |

---

## 3. Labeling Logic: Heuristic Ground Truth

Before the AI can predict, we train it using a **Heuristic Formula** defined in `recompute_routing_cost.py`. This formula represents human expert knowledge of "Good" vs "Bad" paths.

### The Cost Formula (Minimize this Value)
The total `Routing_Cost` is the sum of five component costs:

1.  **Hop Cost:** `Grad * 10.0`
    - Constant penalty per hop to favor shorter paths.
2.  **Signal Cost:** `max(0.0, (-RSSI - 70.0)) * 2.5`
    - Free zone above -70dBm. Every dBm below -70 incurs a linear penalty.
3.  **Reliability Cost:** `min(400.0, pow(Drop_Rate, 1.5))`
    - Exponential growth. A 10% drop rate is bad, but 20% is catastrophically worse.
4.  **Battery Cost:**
    - `200.0` if **Pin < 20%** (Emergency avoidance).
    - `(60.0 - Pin) * 1.5` if **Pin < 60%** (Tapered caution).
    - `0.0` if **Pin >= 60%** (Safe zone).
5.  **Stability Cost:** `100.0 * exp(-Link_UP / 300.0)`
    - High penalty for new links, decays rapidly after 5 minutes (300s).

---

## 4. Operational Data Flow

Detailed breakdown of how a single data point reaches the training set:

### Phase 1: Edge Sampling (Sensor Node)
- **Monitoring:** `packet_stats.c` runs atomically using Zephyr's `atomic_t` counters to track every packet received/sent/dropped without blocking the mesh stack.
- **Trigger:** Gateway sends `mesh topo_req <seq>`.
- **Snapshot:** Node calls `topo_take_snapshot()`. It freezes the counters, calculates `total_sent` for battery estimation, and resets hardware stats via `bt_mesh_stat_reset()`.

### Phase 2: Drip-Feed Transmission
- **Pagination:** To prevent flooding the Mesh network with large topology logs, the data is split into **pages** (4 neighbors per page).
- **Jitter:** Node waits a random interval (up to 8s) before starting, then sends one page every ~300ms.
- **Frame Format:** `$[TOPO],Origin,Seq,TotalPg,CurPg,Count,Grad,Parent,Drops,FwdRate,Uptime,TotalSent,[nb_addr,rssi,nb_grad,nb_uptime]`

### Phase 3: Gateway Ingestion
- **UART Parsing:** `Gateway_main.py` uses high-performance Regex to extract the 11-group header.
- **Assembly:** Reconstructs the full topology graph.
- **Logging:** Writes data to RAM Disk (`/dev/shm` or local CSV) to minimize IO latency on Raspberry Pi.

---

## 5. Model Architecture (LightGBM)

- **Boosting Type:** GBDT (Gradient Boosting Decision Tree).
- **Depth Strategy:** `max_depth=4` and `num_leaves=15`. Shallow trees are used to ensure inference on Raspberry Pi takes **< 1ms**.
- **Histogram-based splits:** Features are binned into buckets (e.g., RSSI thresholds). This makes the model robust against minor noise in radio signal fluctuation.
- **Learning Objective:** MSE (Mean Squared Error) for regression, aiming to match the Heuristic Cost with a relative error **< 5%**.
