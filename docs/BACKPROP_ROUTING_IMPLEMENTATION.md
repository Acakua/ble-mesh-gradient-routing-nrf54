# Hướng Dẫn Triển Khai Backpropagation Routing

## Tổng Quan

Tài liệu này mô tả chi tiết quá trình implement tính năng Backpropagation Routing cho hệ thống BLE Mesh Gradient Routing trên nRF54L15. Tính năng này cho phép Gateway gửi dữ liệu xuống bất kỳ node nào trong mạng (downlink), bổ sung cho tính năng uplink đã có sẵn (node gửi dữ liệu lên Gateway).

## Vấn Đề Cần Giải Quyết

Trước khi implement, hệ thống chỉ hỗ trợ một chiều:
- Node có thể gửi DATA lên Gateway thông qua gradient routing (đi theo hướng gradient giảm dần)
- Gateway **KHÔNG THỂ** gửi dữ liệu xuống một node cụ thể

**Lý do**: Gradient routing chỉ biết "hướng lên" (về phía sink), không biết "hướng xuống" (về phía các node xa hơn).

## Giải Pháp: Backpropagation Routing

**Ý tưởng chính**: Khi nhận DATA từ một node, học lại đường đi ngược (reverse route) để sau này có thể gửi BACKPROP_DATA theo đường đó.

---

## Chi Tiết Implementation Theo Từng Giai Đoạn

---

### Giai đoạn 1: Thêm `original_source` vào DATA Packet

---

#### 1.1. Mục Tiêu

Sửa đổi format của DATA packet để mang theo thông tin về node nguồn gốc (original source), giúp các node trung gian và Gateway biết được packet đến từ node nào ban đầu, bất kể packet đã đi qua bao nhiêu hop trung gian.

---

#### 1.2. Phân Tích Vấn Đề Hiện Tại

**Cấu trúc DATA packet hiện tại:**
```
┌─────────────────┐
│   data (2 bytes)│
└─────────────────┘
         │
         ▼
    Tổng: 2 bytes
```

**Vấn đề:**
- Khi Node C gửi DATA, packet chỉ chứa payload (2 bytes)
- Khi Node B nhận và forward lên Node A, Node A chỉ biết packet đến từ Node B
- Node A **KHÔNG BIẾT** packet thực sự xuất phát từ Node C
- Gateway cũng không biết ai là nguồn gốc thực sự của packet

**Ví dụ minh họa vấn đề:**
```
Node C (gradient=3) ──DATA──► Node B (gradient=2) ──DATA──► Gateway (gradient=0)
       │                              │                           │
       │                              │                           │
  Gửi data=0x1234               Nhận từ C                   Chỉ biết packet
  Không kèm src                 Forward lên                 đến từ B, không
                                Gateway                     biết nguồn gốc là C
```

---

#### 1.3. Giải Pháp Thiết Kế

**Cấu trúc DATA packet mới:**
```
┌──────────────────────┬─────────────────┐
│ original_source (2B) │   data (2 bytes)│
└──────────────────────┴─────────────────┘
         │                      │
         ▼                      ▼
    Địa chỉ node           Payload data
    tạo ra packet
    ban đầu

    Tổng: 4 bytes
```

**Nguyên tắc hoạt động:**
1. Node tạo packet: Điền địa chỉ của chính mình vào `original_source`
2. Node trung gian: **GIỮ NGUYÊN** `original_source`, chỉ forward packet
3. Gateway nhận: Đọc `original_source` để biết packet từ node nào

**Ví dụ sau khi sửa:**
```
Node C (addr=0x0003)                Node B (addr=0x0002)                 Gateway
        │                                   │                               │
        │ DATA [src=0x0003, data=0x1234]   │                               │
        ├──────────────────────────────────►│                               │
        │                                   │                               │
        │                                   │ DATA [src=0x0003, data=0x1234]│
        │                                   ├───────────────────────────────►│
        │                                   │                               │
        │                                   │                    Biết packet từ
        │                                   │                    Node C (0x0003)
```

---

#### 1.4. Chi Tiết Thay Đổi Code

##### 1.4.1. File `include/data_forward.h`

**Mục đích:** Cập nhật prototype các hàm để nhận thêm tham số `original_source`.

**Thay đổi cụ thể:**

```c
/* TRƯỚC */
int data_forward_send(struct bt_mesh_gradient_srv *srv, uint16_t data);

/* SAU */
/**
 * @brief Gửi DATA packet với original_source được chỉ định
 *
 * @param srv       Con trỏ đến gradient server instance
 * @param data      Dữ liệu payload (2 bytes)
 * @param original_source  Địa chỉ của node tạo ra packet ban đầu
 *
 * @return 0 nếu thành công, mã lỗi âm nếu thất bại
 *
 * @note Hàm này được sử dụng khi forward packet từ node khác,
 *       giữ nguyên original_source ban đầu
 */
int data_forward_send(struct bt_mesh_gradient_srv *srv, uint16_t data, 
                      uint16_t original_source);
```

```c
/* TRƯỚC */
int data_forward_send_direct(struct bt_mesh_gradient_srv *srv, uint16_t data);

/* SAU */
/**
 * @brief Gửi DATA packet mới từ chính node này
 *
 * @param srv   Con trỏ đến gradient server instance
 * @param data  Dữ liệu payload (2 bytes)
 *
 * @return 0 nếu thành công, mã lỗi âm nếu thất bại
 *
 * @note Hàm này tự động điền địa chỉ của node hiện tại làm original_source
 *       Sử dụng khi node muốn gửi dữ liệu của chính nó (không phải forward)
 */
int data_forward_send_direct(struct bt_mesh_gradient_srv *srv, uint16_t data);
```

---

##### 1.4.2. File `src/data_forward.c`

**Mục đích:** Implement logic gửi packet với format mới 4 bytes.

**Hàm `data_send_internal()` - Hàm nội bộ gửi packet:**

```c
/**
 * @brief Hàm nội bộ thực hiện gửi DATA packet
 *
 * @param srv             Gradient server instance
 * @param data            Payload data (2 bytes)
 * @param original_source Địa chỉ node nguồn gốc (2 bytes)
 *
 * Cấu trúc packet được gửi:
 * ┌─────────────────────────┬─────────────────┐
 * │ original_source (2B LE) │ data (2B LE)    │
 * └─────────────────────────┴─────────────────┘
 * Byte:  0        1           2        3
 *
 * LE = Little Endian (byte thấp trước)
 */
static int data_send_internal(struct bt_mesh_gradient_srv *srv,
                              uint16_t data,
                              uint16_t original_source)
{
    /* Find best nexthop from forwarding table */
    const neighbor_entry_t *best = nt_best(
        (const neighbor_entry_t *)srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

    if (best == NULL) {
        LOG_WRN("No route available");
        return -ENETUNREACH;
    }

    uint16_t nexthop = best->addr;

    /* Chuẩn bị buffer cho message */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_DATA, 4);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_DATA);

    /* Đóng gói packet: original_source trước, data sau */
    /* Sử dụng Little Endian (LE) theo chuẩn Bluetooth */
    net_buf_simple_add_le16(&msg, original_source);  /* Bytes 0-1 */
    net_buf_simple_add_le16(&msg, data);             /* Bytes 2-3 */

    /* Thiết lập context gửi tin nhắn */
    struct bt_mesh_msg_ctx ctx = {
        .addr = nexthop,           /* Địa chỉ đích (node tiếp theo) */
        .app_idx = srv->model->keys[0],  /* Application key index */
        .send_ttl = BT_MESH_TTL_DEFAULT, /* TTL mặc định */
    };

    LOG_INF("DATA send: original_src=0x%04X, data=0x%04X -> nexthop=0x%04X",
            original_source, data, nexthop);

    /* Gửi message qua BLE Mesh */
    return bt_mesh_model_send(srv->model, &ctx, &msg, NULL, NULL);
}
```

**Hàm `data_forward_send()` - Forward packet từ node khác:**

```c
/**
 * @brief Forward DATA packet, giữ nguyên original_source
 *
 * Được gọi khi node trung gian nhận DATA và cần forward lên
 * hướng Gateway. Original_source được giữ nguyên từ packet nhận được.
 *
 * Flow:
 * Node C ──[src=C]──► Node B (gọi hàm này) ──[src=C]──► Node A
 *                           │
 *                           └── Giữ nguyên src=C, không đổi thành B
 */
int data_forward_send(struct bt_mesh_gradient_srv *srv, uint16_t data,
                      uint16_t original_source, uint16_t sender_addr)
{
    if (!srv) {
        return -EINVAL;
    }

    /* Check if route is available */
    const neighbor_entry_t *best = nt_best(
        (const neighbor_entry_t *)srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

    if (best == NULL) {
        LOG_ERR("[Forward] No route available!");
        return -ENETUNREACH;
    }

    /* FIX: Check if retry context is busy to prevent data race */
    if (data_send_ctx.active) {
        LOG_WRN("[Forward] System busy retrying previous packet, dropping new data %d", data);
        return -EBUSY;
    }

    /* Indicate data forwarding */
    led_indicate_data_forwarded();

    /* Setup context for retry */
    data_send_ctx.gradient_srv = srv;
    data_send_ctx.data = data;
    data_send_ctx.original_source = original_source;
    data_send_ctx.sender_addr = sender_addr;
    data_send_ctx.current_index = 0;
    data_send_ctx.active = true;

    /* Find first valid destination (skip sender) */
    while (data_send_ctx.current_index < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE) {
        const neighbor_entry_t *entry = nt_get(
            (const neighbor_entry_t *)srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
            data_send_ctx.current_index);

        if (entry == NULL) {
            LOG_ERR("[Forward] No valid destination!");
            data_send_ctx.active = false;
            return -ENETUNREACH;
        }

        uint16_t dest_addr = entry->addr;

        if (dest_addr == sender_addr) {
            data_send_ctx.current_index++;
            continue;
        }

        LOG_INF("[Forward] Forwarding: original_src=0x%04x, data=%d, to=0x%04x (index %d)",
                original_source, data, dest_addr, data_send_ctx.current_index);

        int err = data_send_internal(srv, dest_addr, original_source, data);

        if (err) {
            LOG_ERR("[Forward] Failed to queue, err=%d", err);
            data_send_ctx.current_index++;
            continue;
        }

        return 0;
    }

    /* No valid destination found */
    data_send_ctx.active = false;
    LOG_ERR("[Forward] No valid destination found after checking all entries");
    return -ENETUNREACH;
}
```

**Hàm `data_forward_send_direct()` - Gửi packet mới từ node này:**

```c
/**
 * @brief Gửi DATA packet mới, tự điền original_source = địa chỉ node hiện tại
 *
 * Được gọi khi node muốn gửi dữ liệu của chính nó lên Gateway
 * (ví dụ: dữ liệu sensor, button press, heartbeat)
 *
 * Flow:
 * Node C (addr=0x0003) gọi hàm này với data=0x1234
 *     │
 *     ▼
 * Packet được tạo: [original_source=0x0003, data=0x1234]
 *     │
 *     ▼
 * Gửi lên nexthop theo gradient routing
 */
int data_forward_send_direct(struct bt_mesh_gradient_srv *srv, uint16_t data)
{
    if (!srv || !srv->model) {
        return -EINVAL;
    }

    /* Get my own address as original_source (I am creating this packet) */
    uint16_t my_addr = bt_mesh_model_elem(srv->model)->rt->addr;

    /* Find best nexthop from forwarding table */
    const neighbor_entry_t *best = nt_best(
        (const neighbor_entry_t *)srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

    if (best == NULL) {
        LOG_WRN("[Direct] No route available in forwarding table!");
        return -ENETUNREACH;
    }

    /* FIX: Check if retry context is busy to prevent data race */
    if (data_send_ctx.active) {
        LOG_WRN("[Direct] System busy retrying previous packet, dropping new data %d", data);
        return -EBUSY;
    }

    uint16_t nexthop = best->addr;

    data_send_ctx.gradient_srv = srv;
    data_send_ctx.data = data;
    data_send_ctx.original_source = my_addr;
    data_send_ctx.sender_addr = BT_MESH_ADDR_UNASSIGNED;
    data_send_ctx.current_index = 0;
    data_send_ctx.active = true;

    LOG_INF("[Direct] Sending data %d with original_src=0x%04x to nexthop=0x%04x",
            data, my_addr, nexthop);

    return data_send_internal(srv, nexthop, my_addr, data);
}
```

---

##### 1.4.3. File `src/gradient_srv.c`

**Mục đích:** Sửa handler để parse packet format mới và forward đúng cách.

**Hàm `handle_data_message()` - Xử lý khi nhận DATA:**

```c
/**
 * @brief Handler xử lý DATA message nhận được
 *
 * @param model   Model nhận message
 * @param ctx     Context chứa thông tin sender, TTL, etc.
 * @param buf     Buffer chứa payload của message
 *
 * Cấu trúc packet nhận được:
 * ┌─────────────────────────┬─────────────────┐
 * │ original_source (2B LE) │ data (2B LE)    │
 * └─────────────────────────┴─────────────────┘
 *
 * Logic xử lý:
 * 1. Parse packet: đọc original_source và data
 * 2. Log thông tin nhận được
 * 3. Nếu là Gateway (gradient=0): Đã đến đích, xử lý data
 * 4. Nếu không phải Gateway: Forward lên nexthop, giữ nguyên original_source
 */
static int handle_data_message(const struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *srv = model->rt->user_data;

    /* Kiểm tra độ dài packet (phải đủ 4 bytes) */
    if (buf->len < 4) {
        LOG_ERR("DATA packet too short: %d bytes (expected 4)", buf->len);
        return -EINVAL;
    }

    /* Parse packet theo thứ tự: original_source trước, data sau */
    uint16_t original_source = net_buf_simple_pull_le16(buf);  /* Bytes 0-1 */
    uint16_t data = net_buf_simple_pull_le16(buf);             /* Bytes 2-3 */

    /* Log thông tin nhận được */
    LOG_INF("DATA received: sender=0x%04X, original_src=0x%04X, data=0x%04X",
            ctx->addr,           /* Node vừa gửi packet đến (có thể là trung gian) */
            original_source,     /* Node tạo ra packet ban đầu */
            data);

    /* Kiểm tra xem đã đến đích chưa (Gateway có gradient = 0) */
    if (srv->gradient == 0) {
        /* Đây là Gateway - đã đến đích cuối cùng */
        LOG_INF("Gateway received DATA from node 0x%04X: 0x%04X", 
                original_source, data);

        /* TODO (giai đoạn 3): Học reverse route từ packet này */
        /* rrt_add_dest(table, ctx->addr, original_source, k_uptime_get()); */

        /* Gọi callback để xử lý data (nếu có) */
        if (srv->handlers && srv->handlers->data_received) {
            srv->handlers->data_received(srv, original_source, data);
        }

        return 0;
    }

    /* Không phải Gateway - cần forward lên */
    LOG_DBG("Forwarding DATA: original_src=0x%04X, data=0x%04X", 
            original_source, data);

    /* TODO (giai đoạn 3): Học reverse route từ packet này */
    /* rrt_add_dest(table, ctx->addr, original_source, k_uptime_get()); */

    /* Forward packet, GIỮ NGUYÊN original_source */
    return data_forward_send(srv, data, original_source);
}
```

---

#### 1.5. Ví Dụ Hoạt Động Chi Tiết

**Scenario: Node C (0x0003) gửi data=0x1234 lên Gateway qua Node B (0x0002)**

```
Bước 1: Node C tạo packet
────────────────────────
Node C (addr=0x0003, gradient=3) gọi:
    data_forward_send_direct(srv, 0x1234)

Hàm tự động điền:
    original_source = 0x0003 (địa chỉ của C)

Packet được tạo:
    ┌──────────┬──────────┐
    │  0x0003  │  0x1234  │
    └──────────┴──────────┘
     original    data
     source

Gửi đến nexthop = 0x0002 (Node B)


Bước 2: Node B nhận và forward
──────────────────────────────
Node B (addr=0x0002, gradient=2) nhận packet

handle_data_message() được gọi:
    - ctx->addr = 0x0003 (sender)
    - original_source = 0x0003 (parse từ packet)
    - data = 0x1234

Vì gradient=2 != 0, cần forward:
    data_forward_send(srv, 0x1234, 0x0003)
                                    │
                                    └── GIỮ NGUYÊN original_source

Packet forward:
    ┌──────────┬──────────┐
    │  0x0003  │  0x1234  │  ← Vẫn là 0x0003, không đổi thành 0x0002
    └──────────┴──────────┘

Gửi đến nexthop = 0x0001 (Gateway)


Bước 3: Gateway nhận packet
───────────────────────────
Gateway (addr=0x0001, gradient=0) nhận packet

handle_data_message() được gọi:
    - ctx->addr = 0x0002 (sender - Node B)
    - original_source = 0x0003 (Node C - nguồn gốc thực sự)
    - data = 0x1234

Vì gradient=0, đây là đích:
    LOG: "Gateway received DATA from node 0x0003: 0x1234"

Gateway biết:
    - Packet đến từ Node C (0x0003) - qua original_source
    - Packet được forward bởi Node B (0x0002) - qua ctx->addr
```

---

#### 1.6. Tóm Tắt Thay Đổi

| File | Thay Đổi |
|------|----------|
| `include/data_forward.h` | Thêm tham số `original_source` vào `data_forward_send()` |
| `src/data_forward.c` | Sửa `data_send_internal()` gửi 4 bytes thay vì 2 bytes |
| `src/data_forward.c` | Sửa `data_forward_send()` nhận và truyền `original_source` |
| `src/data_forward.c` | Sửa `data_forward_send_direct()` tự điền địa chỉ node |
| `src/gradient_srv.c` | Sửa `handle_data_message()` parse 4 bytes |
| `src/gradient_srv.c` | Forward giữ nguyên `original_source` |

---

#### 1.7. Kết Quả

Sau khi implement giai đoạn 1:

✅ Mỗi DATA packet mang theo địa chỉ của node tạo ra nó  
✅ Các node trung gian giữ nguyên `original_source` khi forward  
✅ Gateway biết chính xác packet xuất phát từ node nào  
✅ Chuẩn bị sẵn sàng cho giai đoạn 3: học reverse route từ `original_source`

---

#### 1.8. Lưu Ý Quan Trọng

1. **Backward Compatibility:** Format packet mới (4 bytes) KHÔNG tương thích với format cũ (2 bytes). Tất cả node trong mạng phải được update cùng lúc.

2. **Memory:** Tăng 2 bytes cho mỗi DATA packet. Với BLE Mesh MTU ~11 bytes cho unsegmented message, vẫn đủ không gian.

3. **Không thay đổi Opcode:** Vẫn sử dụng `BT_MESH_GRADIENT_SRV_OP_DATA`, chỉ thay đổi format payload.

4. **Little Endian:** Tuân theo chuẩn Bluetooth, sử dụng Little Endian cho multi-byte values.

---

### giai đoạn 2: Tạo Cấu Trúc Reverse Routing Table (RRT)

---

#### 2.1. Mục Tiêu

Tạo cấu trúc dữ liệu để lưu trữ thông tin reverse route - "để đến node X, gửi qua neighbor Y". Đây là nền tảng để Gateway và các node trung gian có thể gửi BACKPROP packet xuống các node cụ thể.

---

#### 2.2. Phân Tích Yêu Cầu

**Câu hỏi cần trả lời:** Khi Gateway muốn gửi packet đến Node C, nó phải gửi qua node nào?

**Thông tin cần lưu trữ:**
- **Destination:** Node đích cần đến (ví dụ: Node C - 0x0003)
- **Nexthop:** Node kế tiếp để đến destination (ví dụ: Node A - 0x0001)
- **Timestamp:** Thời điểm cuối cùng nhận packet từ destination (để xác định route còn valid không)

**Ví dụ:**
```
Gateway's Reverse Routing Table:
┌─────────────┬─────────────┬─────────────────┐
│ Destination │   Nexthop   │    Last Seen    │
├─────────────┼─────────────┼─────────────────┤
│   0x0003    │   0x0002    │   12345 ms      │  ← Để đến Node C, gửi qua Node B
│   0x0004    │   0x0002    │   12300 ms      │  ← Để đến Node D, gửi qua Node B
│   0x0005    │   0x0006    │   12100 ms      │  ← Để đến Node E, gửi qua Node F
└─────────────┴─────────────┴─────────────────┘
```

---

#### 2.3. Thiết Kế Cấu Trúc Dữ Liệu

**Phương án được chọn: Linked List gắn với Forwarding Table**

Thay vì tạo một bảng RRT riêng biệt, ta gắn danh sách destinations vào mỗi neighbor entry trong forwarding table đã có sẵn.

**Lý do:**
1. **Tận dụng cấu trúc có sẵn:** Forwarding table đã quản lý danh sách neighbors
2. **Tra cứu nhanh:** Khi tìm nexthop, chỉ cần duyệt neighbors đã biết
3. **Cleanup đồng bộ:** Khi neighbor bị xóa, các destination của nó cũng tự động bị xóa
4. **Tiết kiệm memory:** Không cần duplicate thông tin neighbor

**Cấu trúc:**
```
Forwarding Table (đã có sẵn)
┌────────────────────────────────────────────────────────────┐
│ neighbor_entry_t [0]                                       │
│   ├── addr: 0x0002 (Node B)                               │
│   ├── gradient: 1                                          │
│   ├── last_seen: 12000 ms                                  │
│   └── backprop_dest ──► [Linked List của destinations]     │
│                              │                             │
│                              ▼                             │
│                         ┌─────────────┐                    │
│                         │ addr: 0x0003│                    │
│                         │ last_seen   │                    │
│                         │ next ───────┼──►┌─────────────┐  │
│                         └─────────────┘   │ addr: 0x0004│  │
│                                           │ last_seen   │  │
│                                           │ next: NULL  │  │
│                                           └─────────────┘  │
├────────────────────────────────────────────────────────────┤
│ neighbor_entry_t [1]                                       │
│   ├── addr: 0x0006 (Node F)                               │
│   ├── gradient: 2                                          │
│   └── backprop_dest ──► [0x0005] ──► NULL                  │
└────────────────────────────────────────────────────────────┘
```

---

#### 2.4. Chi Tiết Thay Đổi Code

##### 2.4.1. File `include/gradient_types.h`

**Mục đích:** Thêm con trỏ `backprop_dest` vào struct `neighbor_entry_t`.

```c
/* Forward declaration cho backprop_node */
struct backprop_node;

/**
 * @brief Entry trong Forwarding Table
 *
 * Mỗi entry đại diện cho một neighbor node mà node hiện tại
 * có thể giao tiếp trực tiếp (1-hop).
 */
typedef struct neighbor_entry {
    uint16_t addr;           /**< Địa chỉ unicast của neighbor */
    uint8_t gradient;        /**< Gradient value của neighbor */
    int64_t last_seen;       /**< Timestamp lần cuối nhận message từ neighbor */

    /**
     * @brief Danh sách các destination có thể đến qua neighbor này
     *
     * Linked list chứa các node mà ta có thể reach được
     * bằng cách gửi qua neighbor này.
     *
     * Ví dụ: Nếu neighbor là Node B, và backprop_dest chứa [C, D],
     * nghĩa là để gửi đến Node C hoặc D, ta gửi qua Node B.
     *
     * Được sử dụng cho Backpropagation Routing (downlink).
     */
    struct backprop_node *backprop_dest;
} neighbor_entry_t;
```

---

##### 2.4.2. File `include/reverse_routing.h` (FILE MỚI)

**Mục đích:** Định nghĩa struct và API cho Reverse Routing Table.

```c
/**
 * @file reverse_routing.h
 * @brief Reverse Routing Table cho Backpropagation Routing
 *
 * Module này quản lý thông tin reverse route - cho biết
 * để đến một destination cụ thể, cần gửi qua neighbor nào.
 *
 * Cấu trúc: Mỗi neighbor trong forwarding table có một linked list
 * chứa các destination mà nó có thể dẫn tới.
 */

#ifndef REVERSE_ROUTING_H_
#define REVERSE_ROUTING_H_

#include <zephyr/kernel.h>
#include "gradient_types.h"

/**
 * @brief Node trong linked list của destinations
 *
 * Mỗi node đại diện cho một destination có thể reach được
 * thông qua một neighbor cụ thể.
 */
typedef struct backprop_node {
    uint16_t addr;              /**< Địa chỉ của destination */
    int64_t last_seen;          /**< Thời điểm cuối nhận packet từ dest này */
    struct backprop_node *next; /**< Con trỏ đến node tiếp theo trong list */
} backprop_node_t;

/**
 * @brief Khởi tạo Reverse Routing Table
 *
 * @param table  Con trỏ đến forwarding table
 *
 * Đảm bảo tất cả backprop_dest pointers được set về NULL.
 */
void rrt_init(neighbor_entry_t *table);

/**
 * @brief Thêm hoặc cập nhật destination trong RRT
 *
 * @param table     Con trỏ đến forwarding table
 * @param nexthop   Địa chỉ của neighbor (node kế tiếp)
 * @param dest      Địa chỉ của destination cần thêm
 * @param timestamp Thời điểm hiện tại (từ k_uptime_get())
 *
 * @return 0 nếu thành công, mã lỗi âm nếu thất bại
 *
 * Logic:
 * 1. Nếu dest đã tồn tại ở nexthop KHÁC → xóa khỏi nexthop cũ trước
 * 2. Nếu dest đã tồn tại ở nexthop NÀY → chỉ update last_seen
 * 3. Nếu dest chưa tồn tại → tạo node mới, thêm vào linked list
 *
 * Trường hợp 1 xử lý khi route thay đổi (node di chuyển hoặc
 * path tốt hơn được phát hiện).
 */
int rrt_add_dest(neighbor_entry_t *table, uint16_t nexthop,
                 uint16_t dest, int64_t timestamp);

/**
 * @brief Xóa một destination khỏi RRT
 *
 * @param table  Con trỏ đến forwarding table
 * @param dest   Địa chỉ destination cần xóa
 *
 * @return 0 nếu tìm thấy và xóa, -ENOENT nếu không tìm thấy
 *
 * Duyệt tất cả neighbors, tìm và xóa destination nếu có.
 */
int rrt_remove_dest(neighbor_entry_t *table, uint16_t dest);

/**
 * @brief Tìm nexthop để đến một destination
 *
 * @param table  Con trỏ đến forwarding table
 * @param dest   Địa chỉ destination cần tìm
 *
 * @return Địa chỉ của nexthop, hoặc BT_MESH_ADDR_UNASSIGNED nếu không tìm thấy
 *
 * Đây là hàm lookup chính được sử dụng khi cần gửi BACKPROP.
 */
uint16_t rrt_find_nexthop(neighbor_entry_t *table, uint16_t dest);

/**
 * @brief Xóa các entry hết hạn trong RRT
 *
 * @param table       Con trỏ đến forwarding table
 * @param timeout_ms  Thời gian timeout (milliseconds)
 *
 * Entry được coi là hết hạn nếu:
 *   current_time - last_seen > timeout_ms
 *
 * Hàm này nên được gọi định kỳ trong cleanup routine.
 */
void rrt_cleanup_expired(neighbor_entry_t *table, int64_t timeout_ms);

/**
 * @brief In nội dung RRT ra log (debug)
 *
 * @param table  Con trỏ đến forwarding table
 *
 * Format output:
 *   === Reverse Routing Table ===
 *   Neighbor 0x0002:
 *     -> 0x0003 (last_seen: 12345 ms ago)
 *     -> 0x0004 (last_seen: 12300 ms ago)
 *   Neighbor 0x0006:
 *     -> 0x0005 (last_seen: 12100 ms ago)
 */
void rrt_print_table(neighbor_entry_t *table);

/**
 * @brief Xóa toàn bộ destinations của một neighbor entry
 *
 * @param entry  Con trỏ đến neighbor entry cần clear
 *
 * Giải phóng tất cả memory của linked list và set
 * backprop_dest = NULL.
 *
 * Được gọi khi neighbor bị xóa khỏi forwarding table.
 */
void rrt_clear_entry(neighbor_entry_t *entry);

/**
 * @brief Lấy một destination bất kỳ từ RRT (cho testing)
 *
 * @param table     Con trỏ đến forwarding table
 * @param out_dest  [out] Địa chỉ destination tìm được
 *
 * @return 0 nếu tìm thấy, -ENOENT nếu RRT rỗng
 *
 * Hàm helper để test - lấy destination đầu tiên trong RRT.
 */
int rrt_get_any_destination(neighbor_entry_t *table, uint16_t *out_dest);

#endif /* REVERSE_ROUTING_H_ */
```

---

##### 2.4.3. File `src/reverse_routing.c` (FILE MỚI)

**Mục đích:** Implement tất cả các hàm API của RRT.

```c
/**
 * @file reverse_routing.c
 * @brief Implementation của Reverse Routing Table
 */

#include "reverse_routing.h"
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/mesh.h>

LOG_MODULE_REGISTER(reverse_routing, CONFIG_BT_MESH_MODEL_LOG_LEVEL);

/* Lấy size của forwarding table từ Kconfig */
#define FWD_TABLE_SIZE CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE

/**
 * @brief Khởi tạo RRT
 */
void rrt_init(neighbor_entry_t *table)
{
    if (!table) {
        return;
    }

    /* Đảm bảo tất cả backprop_dest pointers là NULL */
    for (int i = 0; i < FWD_TABLE_SIZE; i++) {
        table[i].backprop_dest = NULL;
    }

    LOG_INF("RRT initialized");
}

/**
 * @brief Tìm neighbor entry theo địa chỉ
 *
 * @param table    Forwarding table
 * @param addr     Địa chỉ cần tìm
 *
 * @return Con trỏ đến entry nếu tìm thấy, NULL nếu không
 */
static neighbor_entry_t *find_neighbor(neighbor_entry_t *table, uint16_t addr)
{
    for (int i = 0; i < FWD_TABLE_SIZE; i++) {
        if (table[i].addr == addr) {
            return &table[i];
        }
    }
    return NULL;
}

/**
 * @brief Tìm và xóa destination từ một neighbor cụ thể
 *
 * @param entry    Neighbor entry
 * @param dest     Destination cần xóa
 *
 * @return true nếu tìm thấy và xóa, false nếu không tìm thấy
 */
static bool remove_dest_from_entry(neighbor_entry_t *entry, uint16_t dest)
{
    if (!entry || !entry->backprop_dest) {
        return false;
    }

    backprop_node_t *prev = NULL;
    backprop_node_t *curr = entry->backprop_dest;

    while (curr) {
        if (curr->addr == dest) {
            /* Tìm thấy - xóa node này */
            if (prev) {
                prev->next = curr->next;
            } else {
                /* Xóa node đầu tiên */
                entry->backprop_dest = curr->next;
            }
            k_free(curr);
            return true;
        }
        prev = curr;
        curr = curr->next;
    }

    return false;
}

/**
 * @brief Thêm hoặc cập nhật destination (FIXED: Allocate-before-remove)
 *
 * @param table     Con trỏ đến forwarding table
 * @param table_size Kích thước của forwarding table
 * @param nexthop   Địa chỉ của nexthop
 * @param dest      Địa chỉ destination
 * @param timestamp Timestamp hiện tại
 *
 * @return 0 nếu thành công, negative error code nếu thất bại
 *
 * Logic xử lý:
 * 1. Tìm neighbor entry cho nexthop
 * 2. Nếu dest đã tồn tại ở nexthop này: update timestamp
 * 3. Nếu dest tồn tại ở nexthop khác: Move operation (ALLOCATE FIRST)
 * 4. Nếu dest chưa tồn tại: Add operation
 *
 * FIX: Allocate memory FIRST before removing old entries to prevent data loss
 */
int rrt_add_dest(void *table, size_t table_size,
                 uint16_t nexthop_addr, uint16_t dest_addr, int64_t timestamp)
{
    if (!table) {
        return -EINVAL;
    }

    neighbor_entry_t *ft = (neighbor_entry_t *)table;
    neighbor_entry_t *target_entry = NULL;

    /* 1. Find target neighbor entry */
    for (size_t i = 0; i < table_size; i++) {
        if (ft[i].addr == nexthop_addr) {
            target_entry = &ft[i];
            break;
        }
    }

    if (!target_entry) {
        LOG_WRN("[RRT] Nexthop 0x%04x not in forwarding table", nexthop_addr);
        return -ENOENT;
    }

    /* 2. Check if destination already exists at this nexthop */
    backprop_node_t *curr = target_entry->backprop_dest;
    while (curr) {
        if (curr->addr == dest_addr) {
            /* Already exists - just update timestamp */
            curr->last_seen = timestamp;
            LOG_DBG("[RRT] Updated dest 0x%04x via nexthop 0x%04x", dest_addr, nexthop_addr);
            return 0;
        }
        curr = curr->next;
    }

    /* Ensure it doesn't exist elsewhere (duplicate cleanup) */
    for (size_t i = 0; i < table_size; i++) {
         if (&ft[i] != target_entry && ft[i].addr != 0) {
             if (remove_dest_from_list(&ft[i].backprop_dest, dest_addr)) {
                 LOG_WRN("[RRT] Fixed duplicate dest 0x%04x (removed from 0x%04x)", dest_addr, ft[i].addr);
             }
         }
    }

    /* 3. It's a Move or New Add. ALLOCATE FIRST to prevent data loss. */
    backprop_node_t *new_node = k_malloc(sizeof(backprop_node_t));
    if (new_node == NULL) {
        LOG_ERR("[RRT] Failed to allocate memory for backprop node. Keeping old route if any.");
        return -ENOMEM; /* Abort before deleting anything */
    }

    /* 4. Now safe to remove from old location (Move operation) */
    for (size_t i = 0; i < table_size; i++) {
        if (ft[i].addr != nexthop_addr && ft[i].addr != 0) {
            if (remove_dest_from_list(&ft[i].backprop_dest, dest_addr)) {
                LOG_INF("[RRT] Dest 0x%04x moved from nexthop 0x%04x to 0x%04x",
                        dest_addr, ft[i].addr, nexthop_addr);
                break; /* Can only exist in one place */
            }
        }
    }

    /* 5. Check limits and insert */
    size_t count = count_list(target_entry->backprop_dest);
    if (count >= RRT_MAX_DEST_PER_NEXTHOP) {
        LOG_WRN("[RRT] Max destinations reached for nexthop 0x%04x, removing oldest", nexthop_addr);
        remove_oldest_from_list(&target_entry->backprop_dest);
    }

    new_node->addr = dest_addr;
    new_node->last_seen = timestamp;
    new_node->next = target_entry->backprop_dest;
    target_entry->backprop_dest = new_node;

    LOG_INF("[RRT] Added dest 0x%04x via nexthop 0x%04x", dest_addr, nexthop_addr);
    return 0;
}

/**
 * @brief Xóa destination khỏi RRT
 */
int rrt_remove_dest(neighbor_entry_t *table, uint16_t dest)
{
    if (!table) {
        return -EINVAL;
    }

    for (int i = 0; i < FWD_TABLE_SIZE; i++) {
        if (remove_dest_from_entry(&table[i], dest)) {
            LOG_INF("RRT: Removed dest 0x%04X", dest);
            return 0;
        }
    }

    return -ENOENT;
}

/**
 * @brief Tìm nexthop cho destination
 */
uint16_t rrt_find_nexthop(neighbor_entry_t *table, uint16_t dest)
{
    if (!table) {
        return BT_MESH_ADDR_UNASSIGNED;
    }

    /* Duyệt tất cả neighbors */
    for (int i = 0; i < FWD_TABLE_SIZE; i++) {
        if (table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            continue;  /* Entry trống */
        }

        /* Duyệt linked list của neighbor này */
        backprop_node_t *curr = table[i].backprop_dest;
        while (curr) {
            if (curr->addr == dest) {
                LOG_DBG("RRT: Found route to 0x%04X via 0x%04X",
                        dest, table[i].addr);
                return table[i].addr;  /* Trả về địa chỉ neighbor */
            }
            curr = curr->next;
        }
    }

    LOG_WRN("RRT: No route to destination 0x%04X", dest);
    return BT_MESH_ADDR_UNASSIGNED;
}

/**
 * @brief Cleanup entries hết hạn
 */
void rrt_cleanup_expired(neighbor_entry_t *table, int64_t timeout_ms)
{
    if (!table) {
        return;
    }

    int64_t now = k_uptime_get();
    int removed_count = 0;

    for (int i = 0; i < FWD_TABLE_SIZE; i++) {
        if (table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            continue;
        }

        backprop_node_t *prev = NULL;
        backprop_node_t *curr = table[i].backprop_dest;

        while (curr) {
            backprop_node_t *next = curr->next;

            /* Kiểm tra có hết hạn không */
            if ((now - curr->last_seen) > timeout_ms) {
                LOG_INF("RRT: Expired dest 0x%04X (age: %lld ms)",
                        curr->addr, now - curr->last_seen);

                /* Xóa node này */
                if (prev) {
                    prev->next = next;
                } else {
                    table[i].backprop_dest = next;
                }
                k_free(curr);
                removed_count++;
            } else {
                prev = curr;
            }

            curr = next;
        }
    }

    if (removed_count > 0) {
        LOG_INF("RRT: Cleanup removed %d expired entries", removed_count);
    }
}

/**
 * @brief In RRT ra log
 */
void rrt_print_table(neighbor_entry_t *table)
{
    if (!table) {
        return;
    }

    int64_t now = k_uptime_get();
    int total_routes = 0;

    LOG_INF("=== Reverse Routing Table ===");

    for (int i = 0; i < FWD_TABLE_SIZE; i++) {
        if (table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            continue;
        }

        backprop_node_t *curr = table[i].backprop_dest;
        if (!curr) {
            continue;  /* Neighbor không có destinations */
        }

        LOG_INF("Neighbor 0x%04X:", table[i].addr);

        while (curr) {
            int64_t age = now - curr->last_seen;
            LOG_INF("  -> 0x%04X (age: %lld ms)", curr->addr, age);
            total_routes++;
            curr = curr->next;
        }
    }

    if (total_routes == 0) {
        LOG_INF("  (empty)");
    } else {
        LOG_INF("Total: %d routes", total_routes);
    }
}

/**
 * @brief Clear toàn bộ destinations của một entry
 */
void rrt_clear_entry(neighbor_entry_t *entry)
{
    if (!entry) {
        return;
    }

    backprop_node_t *curr = entry->backprop_dest;
    while (curr) {
        backprop_node_t *next = curr->next;
        k_free(curr);
        curr = next;
    }

    entry->backprop_dest = NULL;
}

/**
 * @brief Lấy destination đầu tiên trong RRT
 */
int rrt_get_any_destination(neighbor_entry_t *table, uint16_t *out_dest)
{
    if (!table || !out_dest) {
        return -EINVAL;
    }

    for (int i = 0; i < FWD_TABLE_SIZE; i++) {
        if (table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            continue;
        }

        if (table[i].backprop_dest) {
            *out_dest = table[i].backprop_dest->addr;
            return 0;
        }
    }

    return -ENOENT;  /* RRT rỗng */
}
```

---

##### 2.4.4. File `CMakeLists.txt`

**Mục đích:** Thêm source file mới vào build.

```cmake
# Thêm vào danh sách target_sources
target_sources(app PRIVATE
    src/main.c
    src/model_handler.c
    src/gradient_srv.c
    src/neighbor_table.c
    src/routing_policy.c
    src/led_indication.c
    src/data_forward.c
    src/gradient_work.c
    src/reverse_routing.c    # <-- THÊM DÒNG NÀY
)
```

---

##### 2.4.5. File `src/neighbor_table.c`

**Mục đích:** Tích hợp RRT với neighbor table management.

**Khi tạo entry mới - khởi tạo backprop_dest:**

```c
#include "reverse_routing.h"

/**
 * @brief Thêm hoặc cập nhật neighbor entry
 */
int neighbor_table_add(neighbor_entry_t *table, uint16_t addr,
                       uint8_t gradient, int64_t timestamp)
{
    /* ... existing code để tìm/tạo entry ... */

    /* Nếu tạo entry mới, khởi tạo backprop_dest */
    if (is_new_entry) {
        entry->backprop_dest = NULL;  /* Quan trọng! */
    }

    /* ... */
}
```

**Khi xóa entry - giải phóng memory:**

```c
/**
 * @brief Xóa neighbor entry
 */
int neighbor_table_remove(neighbor_entry_t *table, uint16_t addr)
{
    neighbor_entry_t *entry = find_entry(table, addr);
    if (!entry) {
        return -ENOENT;
    }

    /* Giải phóng linked list của destinations */
    rrt_clear_entry(entry);  /* <-- THÊM DÒNG NÀY */

    /* Reset entry */
    entry->addr = BT_MESH_ADDR_UNASSIGNED;
    entry->gradient = 0;
    entry->last_seen = 0;
    /* backprop_dest đã được clear ở trên */

    return 0;
}
```

---

#### 2.5. Ví Dụ Hoạt Động Chi Tiết

**Scenario: Gateway học routes khi nhận DATA từ nhiều nodes**

```
Trạng thái ban đầu:
──────────────────
Gateway's Forwarding Table:
┌─────┬──────────┬──────────┬─────────────┐
│ Idx │   Addr   │ Gradient │ backprop_dest│
├─────┼──────────┼──────────┼─────────────┤
│  0  │  0x0002  │    1     │    NULL     │  (Node B)
│  1  │  0x0006  │    2     │    NULL     │  (Node F)
│  2  │  (empty) │    -     │    NULL     │
└─────┴──────────┴──────────┴─────────────┘


Bước 1: Nhận DATA từ Node C qua Node B
──────────────────────────────────────
Packet: [original_src=0x0003, data=0x1234]
Sender: Node B (0x0002)

Gọi: rrt_add_dest(table, 0x0002, 0x0003, timestamp)

Kết quả:
┌─────┬──────────┬──────────┬──────────────────────┐
│ Idx │   Addr   │ Gradient │    backprop_dest     │
├─────┼──────────┼──────────┼──────────────────────┤
│  0  │  0x0002  │    1     │ [0x0003] -> NULL     │  ← Thêm Node C
│  1  │  0x0006  │    2     │    NULL              │
│  2  │  (empty) │    -     │    NULL              │
└─────┴──────────┴──────────┴──────────────────────┘


Bước 2: Nhận DATA từ Node D cũng qua Node B
───────────────────────────────────────────
Packet: [original_src=0x0004, data=0x5678]
Sender: Node B (0x0002)

Gọi: rrt_add_dest(table, 0x0002, 0x0004, timestamp)

Kết quả:
┌─────┬──────────┬──────────┬───────────────────────────────┐
│ Idx │   Addr   │ Gradient │        backprop_dest          │
├─────┼──────────┼──────────┼───────────────────────────────┤
│  0  │  0x0002  │    1     │ [0x0004] -> [0x0003] -> NULL  │  ← Thêm Node D
│  1  │  0x0006  │    2     │    NULL                       │
│  2  │  (empty) │    -     │    NULL                       │
└─────┴──────────┴──────────┴───────────────────────────────┘


Bước 3: Nhận DATA từ Node E qua Node F
──────────────────────────────────────
Packet: [original_src=0x0005, data=0x9ABC]
Sender: Node F (0x0006)

Gọi: rrt_add_dest(table, 0x0006, 0x0005, timestamp)

Kết quả:
┌─────┬──────────┬──────────┬───────────────────────────────┐
│ Idx │   Addr   │ Gradient │        backprop_dest          │
├─────┼──────────┼──────────┼───────────────────────────────┤
│  0  │  0x0002  │    1     │ [0x0004] -> [0x0003] -> NULL  │
│  1  │  0x0006  │    2     │ [0x0005] -> NULL              │  ← Thêm Node E
│  2  │  (empty) │    -     │    NULL                       │
└─────┴──────────┴──────────┴───────────────────────────────┘


Bước 4: Gateway muốn gửi BACKPROP đến Node C
────────────────────────────────────────────
Gọi: rrt_find_nexthop(table, 0x0003)

Quá trình:
1. Duyệt neighbor 0x0002:
   - Duyệt list: 0x0004 ≠ 0x0003, tiếp tục
   - Duyệt list: 0x0003 == 0x0003 ✓ FOUND!
2. Return 0x0002

Kết quả: Để gửi đến Node C, Gateway gửi qua Node B (0x0002)
```

---

#### 2.6. Xử Lý Route Thay Đổi

**Scenario: Node C thay đổi path - bây giờ qua Node F thay vì Node B**

```
Trước đó: Node C gửi DATA qua Node B
Bây giờ:  Node C gửi DATA qua Node F (path mới tốt hơn)

Khi Gateway nhận:
  Packet: [original_src=0x0003, data=0xDEF0]
  Sender: Node F (0x0006)  ← KHÁC VỚI TRƯỚC (là 0x0002)

Gọi: rrt_add_dest(table, 0x0006, 0x0003, timestamp)

Logic trong rrt_add_dest():
1. Kiểm tra 0x0003 đã tồn tại ở nexthop KHÁC chưa
   → Tìm thấy ở neighbor 0x0002 → XÓA khỏi 0x0002
2. Thêm 0x0003 vào neighbor 0x0006

Kết quả:
┌─────┬──────────┬───────────────────────────────┐
│ Idx │   Addr   │        backprop_dest          │
├─────┼──────────┼───────────────────────────────┤
│  0  │  0x0002  │ [0x0004] -> NULL              │  ← Node C đã bị xóa
│  1  │  0x0006  │ [0x0003] -> [0x0005] -> NULL  │  ← Node C được thêm vào đây
└─────┴──────────┴───────────────────────────────┘

Route đến Node C được cập nhật tự động!
```

---

#### 2.7. Tóm Tắt Thay Đổi

| File | Loại | Thay Đổi |
|------|------|----------|
| `include/gradient_types.h` | Sửa | Thêm `struct backprop_node *backprop_dest` |
| `include/reverse_routing.h` | **MỚI** | Định nghĩa struct và API |
| `src/reverse_routing.c` | **MỚI** | Implement toàn bộ RRT logic |
| `CMakeLists.txt` | Sửa | Thêm `src/reverse_routing.c` |
| `src/neighbor_table.c` | Sửa | Init và clear backprop_dest |

---

#### 2.8. Kết Quả

Sau khi implement giai đoạn 2:

✅ Có cấu trúc dữ liệu để lưu reverse routes  
✅ API đầy đủ: add, remove, find, cleanup, print  
✅ Xử lý route thay đổi (di chuyển destination giữa các nexthop)  
✅ Quản lý memory động với k_malloc/k_free  
✅ Tích hợp với forwarding table có sẵn  
✅ Sẵn sàng cho giai đoạn 3: populate RRT khi nhận DATA

---

#### 2.9. Lưu Ý Quan Trọng

1. **Memory Management:**
   - Sử dụng `k_malloc()` và `k_free()` của Zephyr
   - Cần enable `CONFIG_HEAP_MEM_POOL_SIZE` trong `prj.conf`
   - Mỗi destination chiếm ~12 bytes (addr + timestamp + pointer)

2. **Thread Safety:**
   - Code hiện tại KHÔNG thread-safe
   - Nếu cần, thêm mutex vào các hàm public

3. **Forwarding Table Dependency:**
   - RRT phụ thuộc vào forwarding table
   - Destination chỉ có thể được thêm nếu nexthop đã có trong forwarding table
   - Khi neighbor bị xóa, tất cả destinations của nó cũng bị xóa

4. **Lookup Complexity:**
   - `rrt_find_nexthop()`: O(N × M) với N = số neighbors, M = trung bình destinations/neighbor
   - Với mạng nhỏ (<50 nodes), performance không đáng lo ngại

---

### giai đoạn 3: Implement Route Learning Từ DATA Packets

---

#### 3.1. Mục Tiêu

Implement logic tự động học reverse route mỗi khi nhận được DATA packet. Đây là bước "kết nối" giữa giai đoạn 1 (DATA mang `original_source`) và giai đoạn 2 (cấu trúc RRT) - biến thông tin trong packet thành entry trong RRT.

---

#### 3.2. Phân Tích Logic Học Route

**Nguyên tắc cơ bản:**
Khi node A nhận DATA từ node B với `original_source = C`, node A biết:
- "Để gửi đến C, gửi qua B" (vì B vừa gửi packet từ C đến A)

**Tại sao điều này đúng?**
```
Uplink path (DATA đi lên):
C ──► ... ──► B ──► A ──► ... ──► Gateway

Khi A nhận packet từ B:
- sender = B (node trực tiếp gửi)
- original_source = C (node tạo ra packet)

Suy ra Downlink path (BACKPROP đi xuống):
A ──► B ──► ... ──► C

Vậy: Từ A, để đến C, gửi qua B ✓
```

**Ví dụ cụ thể:**
```
Topology:
                    ┌─────────┐
                    │ Gateway │ gradient=0
                    │ (0x0001)│
                    └────┬────┘
                         │
                    ┌────┴────┐
                    │ Node A  │ gradient=1
                    │ (0x0002)│
                    └────┬────┘
                         │
                    ┌────┴────┐
                    │ Node B  │ gradient=2
                    │ (0x0003)│
                    └────┬────┘
                         │
                    ┌────┴────┐
                    │ Node C  │ gradient=3
                    │ (0x0004)│
                    └─────────┘

Khi Node C gửi DATA lên Gateway:

Bước 1: C tạo packet [src=0x0004, data=0x1234]
Bước 2: C gửi đến B
        B nhận: sender=0x0004, original_source=0x0004
        B học: "Đến 0x0004, gửi qua 0x0004" (direct neighbor)
        B forward lên A

Bước 3: A nhận: sender=0x0003 (B), original_source=0x0004 (C)
        A học: "Đến 0x0004, gửi qua 0x0003"
        A forward lên Gateway

Bước 4: Gateway nhận: sender=0x0002 (A), original_source=0x0004 (C)
        Gateway học: "Đến 0x0004, gửi qua 0x0002"
        Gateway xử lý data
```

---

#### 3.3. Chi Tiết Thay Đổi Code

##### 3.3.1. File `src/gradient_srv.c`

**Mục đích:** Thêm logic học route vào handler nhận DATA.

**Thêm include:**

```c
#include "reverse_routing.h"
```

**Sửa hàm `handle_data_message()` (ĐÃ CẬP NHẬT VỚI FIXES):**

```c
/**
 * @brief Handler xử lý DATA message nhận được
 *
 * Ngoài việc forward/process data, còn học reverse route
 * để có thể gửi BACKPROP về sau.
 */
static int handle_data_message(const struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *gradient_srv = model->rt->user_data;

    uint16_t sender_addr = ctx->addr;  /* Immediate sender (1-hop neighbor) */
    int8_t rssi = ctx->recv_rssi;      /* RSSI of this packet */
    
    /* Read packet: [original_source: 2 bytes] + [data: 2 bytes] */
    uint16_t original_source = net_buf_simple_pull_le16(buf);
    uint16_t received_data = net_buf_simple_pull_le16(buf);

    LOG_INF("Received DATA: original_src=0x%04x, sender=0x%04x, data=%d", 
            original_source, sender_addr, received_data);

    /* ============================================================
     * Reverse Route Learning
     * 
     * When receiving DATA from sender_addr with original_source:
     * → Learn: "To reach original_source, send via sender_addr"
     * 
     * Special case for Gateway (gradient=0):
     * - Gateway doesn't add higher-gradient nodes to forwarding table
     *   (because it only needs routes TO lower gradient, not FROM higher)
     * - But Gateway DOES need to learn reverse routes for BACKPROP
     * - Solution: Add sender to forwarding table with high gradient (UINT8_MAX)
     *   so it doesn't affect uplink routing but enables reverse route learning
     * ============================================================ */
    
    int64_t now = k_uptime_get();
    
    /* First, ensure sender is in forwarding table (for RRT to work) */
    /* Lock forwarding table for thread-safe access */
    k_mutex_lock(&gradient_srv->forwarding_table_mutex, K_FOREVER);
    
    /* Check if sender already exists */
    bool sender_exists = false;
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        if (gradient_srv->forwarding_table[i].addr == sender_addr) {
            sender_exists = true;
            /* Update last_seen */
            gradient_srv->forwarding_table[i].last_seen = now;
            break;
        }
    }

    /* Nếu sender chưa có trong bảng, thêm vào forwarding table (mọi node đều làm, không chỉ gateway) */
    if (!sender_exists) {
        /* 
         * FIX: Use UINT8_MAX (255) for Gateway instead of 254.
         * This indicates "Unknown Gradient" until a Beacon is received.
         * For regular nodes, we still guess "my_gradient + 1".
         */
        uint8_t entry_gradient = (gradient_srv->gradient == 0) ? UINT8_MAX : gradient_srv->gradient + 1;
        LOG_INF("[Route Learn] Adding sender 0x%04x to forwarding table (gradient=%d)", sender_addr, entry_gradient);
        nt_update_sorted(gradient_srv->forwarding_table,
                         CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                         sender_addr, entry_gradient, rssi, now);
    }
    
    /* Now learn the reverse route */
    int err = rrt_add_dest(gradient_srv->forwarding_table,
                           CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                           sender_addr,      /* nexthop = node that just sent to us */
                           original_source,  /* dest = node that created the packet */
                           now);
    
    if (err == 0) {
        LOG_INF("[Route Learn] Learned: dest=0x%04x via nexthop=0x%04x",
                original_source, sender_addr);
    } else if (err == -ENOENT) {
        LOG_WRN("[Route Learn] Nexthop 0x%04x not in forwarding table, skip learning",
                sender_addr);
    } else {
        LOG_ERR("[Route Learn] Failed to add route, err=%d", err);
    }

    /* Unlock forwarding table */
    k_mutex_unlock(&gradient_srv->forwarding_table_mutex);

    /* If this is sink node, indicate reception and done */
    if (gradient_srv->gradient == 0) {
        led_indicate_sink_received();
        
        /* Check if this is a heartbeat or real data */
        if (received_data == BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER) {
            LOG_INF("[Sink] Heartbeat received from 0x%04x (via 0x%04x)",
                    original_source, sender_addr);
        } else {
            LOG_INF("[Sink] Data received: %d from original source 0x%04x (via 0x%04x)", 
                    received_data, original_source, sender_addr);
        }
        
        /* Debug: Print reverse routing table on Gateway */
        rrt_print_table(gradient_srv->forwarding_table,
                        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);
        return 0;
    }
    
    /* Forward data towards sink - keep original_source unchanged */
    err = data_forward_send(gradient_srv, received_data, original_source, sender_addr);
    if (err) {
        LOG_ERR("[Forward] Failed to forward data, err=%d", err);
    }
    
    return 0;
}
```

---

##### 3.3.2. File `src/model_handler.c`

**Mục đích:** Khởi tạo RRT khi hệ thống start.

**Thêm include:**

```c
#include "reverse_routing.h"
```

**Sửa hàm init:**

```c
/**
 * @brief Khởi tạo model handler
 *
 * Được gọi khi hệ thống khởi động, trước khi BLE Mesh start.
 */
void model_handler_init(void)
{
    /* Khởi tạo các module khác... */

    /*
     * Khởi tạo Reverse Routing Table
     *
     * Đảm bảo tất cả backprop_dest pointers trong forwarding table
     * được set về NULL trước khi sử dụng.
     */
    rrt_init(gradient_srv.fwd_table);

    LOG_INF("Model handler initialized with RRT");
}
```

---

#### 3.4. Ví Dụ Hoạt Động Chi Tiết

**Scenario: Node C gửi DATA lên Gateway, tất cả nodes học route**

```
Topology:
Gateway(0x0001) ←── NodeA(0x0002) ←── NodeB(0x0003) ←── NodeC(0x0004)
   g=0                 g=1                g=2                g=3


═══════════════════════════════════════════════════════════════════════
BƯỚC 1: NodeC tạo và gửi DATA
═══════════════════════════════════════════════════════════════════════

NodeC gọi: data_forward_send_direct(srv, 0x1234)

Packet được tạo:
┌─────────────────────┬─────────────────┐
│ original_src=0x0004 │   data=0x1234   │
└─────────────────────┴─────────────────┘

NodeC tìm nexthop: 0x0003 (NodeB) - có gradient thấp hơn
NodeC gửi packet đến NodeB


═══════════════════════════════════════════════════════════════════════
BƯỚC 2: NodeB nhận và học route
═══════════════════════════════════════════════════════════════════════

NodeB's handle_data_message() được gọi:
  - ctx->addr = 0x0004 (sender = NodeC)
  - original_source = 0x0004 (cũng là NodeC, vì C tạo packet)
  - data = 0x1234

Route Learning:
  rrt_add_dest(fwd_table, 0x0004, 0x0004, timestamp)
                           │        │
                           │        └── destination = NodeC
                           └── nexthop = NodeC (direct neighbor)

NodeB's RRT sau khi học:
┌───────────────────────────────────────────┐
│ Neighbor 0x0004 (NodeC):                  │
│   └── dest 0x0004 (NodeC)                 │
└───────────────────────────────────────────┘

Ý nghĩa: "Từ NodeB, để đến NodeC, gửi trực tiếp cho NodeC"

NodeB forward packet lên NodeA (gradient thấp hơn)


═══════════════════════════════════════════════════════════════════════
BƯỚC 3: NodeA nhận và học route
═══════════════════════════════════════════════════════════════════════

NodeA's handle_data_message() được gọi:
  - ctx->addr = 0x0003 (sender = NodeB)
  - original_source = 0x0004 (NodeC)
  - data = 0x1234

Route Learning:
  rrt_add_dest(fwd_table, 0x0003, 0x0004, timestamp)
                           │        │
                           │        └── destination = NodeC
                           └── nexthop = NodeB

NodeA's RRT sau khi học:
┌───────────────────────────────────────────┐
│ Neighbor 0x0003 (NodeB):                  │
│   └── dest 0x0004 (NodeC)                 │
└───────────────────────────────────────────┘

Ý nghĩa: "Từ NodeA, để đến NodeC, gửi qua NodeB"

NodeA forward packet lên Gateway


═══════════════════════════════════════════════════════════════════════
BƯỚC 4: Gateway nhận và học route
═══════════════════════════════════════════════════════════════════════

Gateway's handle_data_message() được gọi:
  - ctx->addr = 0x0002 (sender = NodeA)
  - original_source = 0x0004 (NodeC)
  - data = 0x1234

Route Learning:
  rrt_add_dest(fwd_table, 0x0002, 0x0004, timestamp)
                           │        │
                           │        └── destination = NodeC
                           └── nexthop = NodeA

Gateway's RRT sau khi học:
┌───────────────────────────────────────────┐
│ Neighbor 0x0002 (NodeA):                  │
│   └── dest 0x0004 (NodeC)                 │
└───────────────────────────────────────────┘

Ý nghĩa: "Từ Gateway, để đến NodeC, gửi qua NodeA"

Gateway xử lý data (đích cuối cùng)


═══════════════════════════════════════════════════════════════════════
KẾT QUẢ: Tất cả nodes đã học reverse route
═══════════════════════════════════════════════════════════════════════

Bây giờ Gateway muốn gửi BACKPROP đến NodeC:

Gateway → NodeA → NodeB → NodeC
   │         │        │       │
   │         │        │       └── Đích cuối cùng
   │         │        └── RRT: "đến 0x0004, qua 0x0004"
   │         └── RRT: "đến 0x0004, qua 0x0003"
   └── RRT: "đến 0x0004, qua 0x0002"

Path được "học" từ DATA uplink, sử dụng cho BACKPROP downlink ✓
```

---

#### 3.5. Xử Lý Các Trường Hợp Đặc Biệt

##### 3.5.1. Sender Không Có Trong Forwarding Table

```c
/*
 * Trường hợp: Nhận DATA từ node không có trong forwarding table
 *
 * Có thể xảy ra khi:
 * 1. Node mới join mạng, chưa gửi GRADIENT message
 * 2. Entry đã bị expired và xóa
 */
int err = rrt_add_dest(table, ctx->addr, original_source, timestamp);
if (err == -ENOENT) {
    /*
     * Không thể học route vì nexthop không trong table.
     * Bỏ qua - route sẽ được học sau khi:
     * - Node gửi GRADIENT message và được thêm vào table
     * - Hoặc packet tiếp theo từ node đó
     */
    LOG_WRN("Sender 0x%04X not in forwarding table, skip learning", ctx->addr);
}
```

##### 3.5.2. Route Đã Tồn Tại

```c
/*
 * Trường hợp: Route đến destination đã có sẵn
 *
 * rrt_add_dest() sẽ:
 * - Nếu nexthop GIỐNG: Chỉ update timestamp (refresh route)
 * - Nếu nexthop KHÁC: Xóa khỏi nexthop cũ, thêm vào nexthop mới
 */

/* Ví dụ: Route đến 0x0004 đã có qua 0x0002 */
/* Nhận packet từ 0x0003 với original_source = 0x0004 */

rrt_add_dest(table, 0x0003, 0x0004, timestamp);

/*
 * Logic trong rrt_add_dest():
 * 1. Tìm 0x0004 trong tất cả neighbors
 * 2. Thấy ở neighbor 0x0002 → XÓA khỏi 0x0002
 * 3. Thêm 0x0004 vào neighbor 0x0003
 *
 * Route được cập nhật tự động khi path thay đổi
 */
```

##### 3.5.3. Heartbeat vs Real Data

```c
/*
 * Heartbeat packet có data = 0xFFFF (HEARTBEAT_MARKER)
 *
 * Xử lý giống nhau về route learning:
 * - Vẫn học reverse route
 * - Timestamp được refresh
 *
 * Khác nhau về logging/callback:
 * - Heartbeat: Log ít hơn, không gọi data callback
 * - Real data: Log đầy đủ, gọi callback để xử lý
 */
bool is_heartbeat = (data == BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER);

/* Route learning: Giống nhau cho cả hai */
rrt_add_dest(table, ctx->addr, original_source, timestamp);

/* Xử lý: Khác nhau */
if (is_heartbeat) {
    LOG_DBG("Heartbeat from 0x%04X", original_source);
    /* Không cần xử lý thêm */
} else {
    LOG_INF("Data from 0x%04X: 0x%04X", original_source, data);
    /* Có thể gọi callback */
}
```

---

#### 3.6. Flow Diagram

```
                         ┌─────────────────────────┐
                         │   Nhận DATA packet      │
                         │   từ BLE Mesh stack     │
                         └───────────┬─────────────┘
                                     │
                                     ▼
                         ┌─────────────────────────┐
                         │   Parse packet:         │
                         │   - original_source     │
                         │   - data                │
                         └───────────┬─────────────┘
                                     │
                                     ▼
                    ┌────────────────────────────────────┐
                    │        ROUTE LEARNING              │
                    │  rrt_add_dest(table, sender,       │
                    │               original_source,     │
                    │               timestamp)           │
                    └────────────────┬───────────────────┘
                                     │
                              ┌──────┴──────┐
                              │             │
                         err == 0      err != 0
                              │             │
                              ▼             ▼
                    ┌─────────────┐  ┌─────────────┐
                    │Route learned│  │Log warning, │
                    │successfully │  │continue     │
                    └──────┬──────┘  └──────┬──────┘
                           │                │
                           └───────┬────────┘
                                   │
                                   ▼
                         ┌─────────────────────────┐
                         │   Kiểm tra gradient     │
                         └───────────┬─────────────┘
                                     │
                          ┌──────────┴──────────┐
                          │                     │
                    gradient == 0         gradient > 0
                          │                     │
                          ▼                     ▼
               ┌─────────────────────┐  ┌─────────────────────┐
               │   GATEWAY           │  │   INTERMEDIATE      │
               │   - Log data        │  │   - Forward packet  │
               │   - Print RRT       │  │     lên nexthop     │
               │   - Process data    │  │                     │
               └─────────────────────┘  └─────────────────────┘
```

---

#### 3.7. Tóm Tắt Thay Đổi

| File | Thay Đổi |
|------|----------|
| `src/gradient_srv.c` | Thêm `#include "reverse_routing.h"` |
| `src/gradient_srv.c` | Thêm `rrt_add_dest()` trong `handle_data_message()` |
| `src/gradient_srv.c` | Thêm debug logging cho route learning |
| `src/model_handler.c` | Thêm `#include "reverse_routing.h"` |
| `src/model_handler.c` | Gọi `rrt_init()` trong init function |

---

#### 3.8. Kết Quả

Sau khi implement giai đoạn 3:

✅ Route được học tự động khi nhận DATA packet  
✅ Tất cả nodes trên đường đi đều học được reverse route  
✅ Route được refresh mỗi khi nhận packet (cập nhật timestamp)  
✅ Route thay đổi tự động khi path thay đổi  
✅ Xử lý được các edge cases (sender không trong table, heartbeat)  
✅ Gateway có thể in RRT để debug  
✅ Sẵn sàng cho giai đoạn 4: Sử dụng RRT để gửi BACKPROP

---

#### 3.9. Lưu Ý Quan Trọng

1. **Route Learning Chỉ Từ DATA:**
   - Chỉ học route từ DATA packets (bao gồm heartbeat)
   - KHÔNG học từ GRADIENT messages hay control messages khác
   - Lý do: DATA đi qua path thực tế mà ứng dụng quan tâm

2. **Sender Phải Trong Forwarding Table:**
   - `rrt_add_dest()` yêu cầu nexthop phải có trong forwarding table
   - Nếu sender chưa có trong table → route không được học
   - Route sẽ được học ở packet tiếp theo sau khi sender được thêm

3. **Không Có Validation Cho original_source:**
   - Tin tưởng original_source trong packet
   - Không có cơ chế verify nguồn gốc
   - Trong môi trường controlled (lab), điều này chấp nhận được

4. **Memory Consideration:**
   - Mỗi DATA packet có thể tạo 1 RRT entry
   - Với nhiều nodes gửi thường xuyên, RRT có thể lớn
   - Cleanup mechanism (giai đoạn 6) sẽ xóa entries cũ

---

### giai đoạn 4: Tạo BACKPROP_DATA Packet và Handler

---

#### 4.1. Mục Tiêu

Tạo loại packet mới `BACKPROP_DATA` cho downlink communication (Gateway gửi xuống node). Đây là "trái tim" của Backpropagation Routing - packet thực sự mang dữ liệu từ Gateway đến các node trong mạng.

---

#### 4.2. Thiết Kế Packet BACKPROP_DATA

##### 4.2.1. So Sánh Với DATA Packet

| Thuộc Tính | DATA (Uplink) | BACKPROP_DATA (Downlink) |
|------------|---------------|--------------------------|
| Hướng | Node → Gateway | Gateway → Node |
| Routing | Theo gradient (giảm dần) | Theo RRT (reverse routes) |
| Destination | Implicit (Gateway) | Explicit (trong packet) |
| TTL | Không cần | Cần (chống loop) |
| Opcode | 0x0A | 0x0C |

##### 4.2.2. Tại Sao Cần TTL?

**Vấn đề tiềm ẩn: Routing Loop**
```
Scenario loop:
- Gateway gửi BACKPROP đến Node C
- Node A nhận, forward đến Node B (theo RRT)
- Node B nhận, forward đến Node A (RRT cũ/sai)
- Node A nhận lại, forward đến Node B
- ... vòng lặp vô hạn!

Nguyên nhân:
- RRT có thể outdated
- Route thay đổi không đồng bộ
- Topology thay đổi

Giải pháp: TTL
- Mỗi hop giảm TTL đi 1
- Khi TTL = 0, drop packet
- Loop không thể vô hạn
```

##### 4.2.3. Cấu Trúc Packet

```
BACKPROP_DATA Packet Format:
┌─────────────────────┬──────────────┬─────────────────┐
│ final_dest (2 bytes)│ ttl (1 byte) │ payload (2 bytes)│
└─────────────────────┴──────────────┴─────────────────┘
         │                  │                │
         ▼                  ▼                ▼
    Địa chỉ node      Time-to-live      Dữ liệu cần
    đích cuối cùng    (giảm mỗi hop)    gửi đến node

Tổng: 5 bytes

Chi tiết các field:
┌─────────────────────────────────────────────────────────────────┐
│ final_dest (uint16_t, Little Endian)                            │
│   - Địa chỉ unicast của node đích                               │
│   - KHÔNG thay đổi khi forward                                  │
│   - Ví dụ: 0x0004 (Node C)                                      │
├─────────────────────────────────────────────────────────────────┤
│ ttl (uint8_t)                                                   │
│   - Giá trị ban đầu: 10 (DEFAULT_TTL)                           │
│   - Giảm 1 mỗi khi forward                                      │
│   - Drop packet khi <= 1 (MIN_TTL)                              │
│   - Giá trị max: 255 (đủ cho mạng rất lớn)                      │
├─────────────────────────────────────────────────────────────────┤
│ payload (uint16_t, Little Endian)                               │
│   - Dữ liệu thực cần gửi                                        │
│   - Ý nghĩa tùy thuộc vào ứng dụng                              │
│   - Ví dụ: command, sensor config, acknowledgment               │
└─────────────────────────────────────────────────────────────────┘
```

---

#### 4.3. Chi Tiết Thay Đổi Code

##### 4.3.1. File `include/gradient_srv.h`

**Mục đích:** Thêm opcode, constants, và prototypes cho BACKPROP.

```c
/*============================================================
 * BACKPROP_DATA Definitions
 *============================================================*/

/**
 * @brief Opcode cho BACKPROP_DATA message
 *
 * Sử dụng 1-byte opcode trong vendor model opcode space.
 * 0x0C được chọn để không conflict với opcodes hiện có:
 * - 0x0A: DATA
 * - 0x0B: GRADIENT
 */
#define BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA  0x0C

/**
 * @brief TTL mặc định khi tạo BACKPROP packet
 *
 * Giá trị 10 cho phép packet đi qua tối đa 10 hops.
 * Với mạng BLE Mesh thông thường (<10 hops), đây là đủ.
 */
#define BT_MESH_GRADIENT_SRV_BACKPROP_DEFAULT_TTL  10

/**
 * @brief TTL tối thiểu, packet bị drop nếu TTL <= giá trị này
 *
 * Giá trị 1 nghĩa là packet còn có thể forward 1 hop nữa
 * trước khi bị drop. Điều này đảm bảo packet cuối cùng
 * vẫn được deliver nếu đã đến gần đích.
 */
#define BT_MESH_GRADIENT_SRV_BACKPROP_MIN_TTL  1

/**
 * @brief Marker để phân biệt heartbeat với data thực
 *
 * Khi DATA packet có payload = 0xFFFF, đó là heartbeat.
 * Heartbeat dùng để duy trì route, không phải data thực.
 */
#define BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER  0xFFFF

/**
 * @brief Callback khi node nhận được BACKPROP data dành cho nó
 *
 * @param srv      Gradient server instance
 * @param src      Địa chỉ node gửi BACKPROP ban đầu (thường là Gateway)
 * @param data     Dữ liệu payload
 *
 * Callback này được gọi khi:
 * 1. Node nhận BACKPROP_DATA packet
 * 2. final_dest == địa chỉ của node này
 *
 * Ứng dụng implement callback này để xử lý dữ liệu nhận được.
 */
typedef void (*bt_mesh_gradient_srv_data_received_t)(
    struct bt_mesh_gradient_srv *srv,
    uint16_t src,
    uint16_t data);

/**
 * @brief Handlers cho gradient server
 */
struct bt_mesh_gradient_srv_handlers {
    /** Callback khi mesh provisioned và ready */
    bt_mesh_gradient_srv_start_t start;

    /** Callback khi nhận BACKPROP data (MỚI) */
    bt_mesh_gradient_srv_data_received_t data_received;
};

/**
 * @brief Gửi BACKPROP_DATA đến một node cụ thể
 *
 * @param srv      Gradient server instance
 * @param dest     Địa chỉ node đích
 * @param data     Dữ liệu payload cần gửi
 *
 * @return 0 nếu thành công, mã lỗi âm nếu thất bại
 *
 * Hàm này:
 * 1. Tra cứu RRT để tìm nexthop đến dest
 * 2. Tạo BACKPROP_DATA packet với TTL mặc định
 * 3. Gửi packet đến nexthop
 *
 * Thường được gọi từ Gateway để gửi dữ liệu xuống node.
 */
int bt_mesh_gradient_srv_backprop_send(struct bt_mesh_gradient_srv *srv,
                                        uint16_t dest,
                                        uint16_t data);
```

---

##### 4.3.2. File `src/gradient_srv.c`

**Mục đích:** Implement handler và send function cho BACKPROP.

**Thêm include:**

```c
#include "reverse_routing.h"
```

**Handler cho BACKPROP_DATA:**

```c
/**
 * @brief Handler xử lý BACKPROP_DATA message nhận được
 *
 * @param model   Model nhận message
 * @param ctx     Context chứa thông tin sender
 * @param buf     Buffer chứa payload
 *
 * Logic xử lý:
 * 1. Parse packet (final_dest, ttl, payload)
 * 2. Nếu đây là đích (final_dest == my_addr): Deliver locally
 * 3. Nếu TTL quá thấp: Drop packet (chống loop)
 * 4. Nếu không: Forward đến nexthop với TTL giảm đi 1
 */
static int handle_backprop_message(const struct bt_mesh_model *model,
                                   struct bt_mesh_msg_ctx *ctx,
                                   struct net_buf_simple *buf)
{
    struct bt_mesh_gradient_srv *srv = model->rt->user_data;

    /*
     * Bước 1: Kiểm tra và parse packet
     */
    if (buf->len < 5) {
        LOG_ERR("BACKPROP packet too short: %d bytes (expected 5)", buf->len);
        return -EINVAL;
    }

    uint16_t final_dest = net_buf_simple_pull_le16(buf);  /* Bytes 0-1 */
    uint8_t ttl = net_buf_simple_pull_u8(buf);            /* Byte 2 */
    uint16_t payload = net_buf_simple_pull_le16(buf);     /* Bytes 3-4 */

    /* Lấy địa chỉ của node này */
    uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

    LOG_INF("BACKPROP received: sender=0x%04X, dest=0x%04X, ttl=%d, data=0x%04X",
            ctx->addr, final_dest, ttl, payload);

    /*
     * Bước 2: Kiểm tra xem đây có phải là đích không
     */
    if (final_dest == my_addr) {
        /*
         * ╔═══════════════════════════════════════════════════════════╗
         * ║            FINAL DESTINATION REACHED                      ║
         * ╠═══════════════════════════════════════════════════════════╣
         * ║ Packet đã đến đích cuối cùng.                             ║
         * ║ Gọi callback để ứng dụng xử lý dữ liệu.                   ║
         * ╚═══════════════════════════════════════════════════════════╝
         */
        LOG_INF("BACKPROP delivered! src=0x%04X, data=0x%04X",
                ctx->addr, payload);

        /* Gọi callback nếu có */
        if (srv->handlers && srv->handlers->data_received) {
            srv->handlers->data_received(srv, ctx->addr, payload);
        }

        return 0;
    }

    /*
     * Bước 3: Kiểm tra TTL (ĐÃ FIX: Strict check để tránh underflow)
     */
    if (ttl <= BT_MESH_GRADIENT_SRV_BACKPROP_MIN_TTL + 1) {
        /*
         * TTL quá thấp - drop packet để tránh loop
         *
         * FIX: Kiểm tra ttl <= MIN_TTL + 1 để đảm bảo sau khi decrement,
         * TTL vẫn còn valid (>= MIN_TTL) cho next hop.
         */
        LOG_WRN("[BACKPROP] TTL expired (%d <= %d), dropping packet for dest=0x%04x", 
                ttl, BT_MESH_GRADIENT_SRV_BACKPROP_MIN_TTL + 1, final_dest);
        return -ETIMEDOUT;
    }

    /*
     * Bước 4: Forward packet đến nexthop
     */
    
    /* Tìm nexthop từ RRT */
    uint16_t nexthop = rrt_find_nexthop(srv->fwd_table, final_dest);

    if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
        /*
         * Không tìm thấy route đến destination
         *
         * Có thể do:
         * 1. Node chưa bao giờ gửi DATA qua node này
         * 2. Route đã expired và bị xóa
         * 3. Destination không tồn tại
         */
        LOG_WRN("BACKPROP: No route to destination 0x%04X", final_dest);
        return -ENOENT;
    }

    LOG_DBG("BACKPROP forwarding: dest=0x%04X via nexthop=0x%04X, new_ttl=%d",
            final_dest, nexthop, ttl - 1);

    /*
     * Tạo và gửi packet với TTL giảm đi 1
     */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA, 5);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA);

    net_buf_simple_add_le16(&msg, final_dest);  /* Giữ nguyên destination */
    net_buf_simple_add_u8(&msg, ttl - 1);       /* Giảm TTL */
    net_buf_simple_add_le16(&msg, payload);     /* Giữ nguyên payload */

    struct bt_mesh_msg_ctx forward_ctx = {
        .addr = nexthop,
        .app_idx = srv->model->keys[0],
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    return bt_mesh_model_send(srv->model, &forward_ctx, &msg, NULL, NULL);
}
```

**Thêm vào opcode array:**

```c
/**
 * @brief Opcode handlers cho gradient server
 *
 * Mảng này map opcodes với handler functions.
 * BLE Mesh stack sẽ gọi handler tương ứng khi nhận message.
 */
static const struct bt_mesh_model_op _bt_mesh_gradient_srv_op[] = {
    /* DATA message handler */
    {
        BT_MESH_GRADIENT_SRV_OP_DATA,
        BT_MESH_LEN_MIN(4),  /* Minimum 4 bytes: original_src + data */
        handle_data_message
    },
    /* GRADIENT message handler */
    {
        BT_MESH_GRADIENT_SRV_OP_GRADIENT,
        BT_MESH_LEN_MIN(1),  /* Minimum 1 byte: gradient value */
        handle_gradient_message
    },
    /* BACKPROP_DATA message handler (MỚI) */
    {
        BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA,
        BT_MESH_LEN_MIN(5),  /* Minimum 5 bytes: dest + ttl + payload */
        handle_backprop_message
    },
    BT_MESH_MODEL_OP_END,
};
```

**Hàm gửi BACKPROP (được gọi từ application):**

```c
/**
 * @brief Gửi BACKPROP_DATA đến một node cụ thể
 *
 * Đây là API chính để Gateway (hoặc bất kỳ node nào) gửi
 * dữ liệu xuống một node khác trong mạng.
 */
int bt_mesh_gradient_srv_backprop_send(struct bt_mesh_gradient_srv *srv,
                                        uint16_t dest,
                                        uint16_t data)
{
    if (!srv || !srv->model) {
        LOG_ERR("BACKPROP send: Invalid server");
        return -EINVAL;
    }

    /*
     * Bước 1: Tìm nexthop từ RRT
     */
    uint16_t nexthop = rrt_find_nexthop(srv->fwd_table, dest);

    if (nexthop == BT_MESH_ADDR_UNASSIGNED) {
        LOG_ERR("BACKPROP send: No route to 0x%04X", dest);
        return -ENOENT;
    }

    LOG_INF("BACKPROP send: dest=0x%04X, data=0x%04X, nexthop=0x%04X",
            dest, data, nexthop);

    /*
     * Bước 2: Tạo packet
     */
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA, 5);
    bt_mesh_model_msg_init(&msg, BT_MESH_GRADIENT_SRV_OP_BACKPROP_DATA);

    net_buf_simple_add_le16(&msg, dest);                              /* Destination */
    net_buf_simple_add_u8(&msg, BT_MESH_GRADIENT_SRV_BACKPROP_DEFAULT_TTL);  /* TTL */
    net_buf_simple_add_le16(&msg, data);                              /* Payload */

    /*
     * Bước 3: Gửi đến nexthop
     */
    struct bt_mesh_msg_ctx ctx = {
        .addr = nexthop,
        .app_idx = srv->model->keys[0],
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    return bt_mesh_model_send(srv->model, &ctx, &msg, NULL, NULL);
}
```

---

##### 4.3.3. File `src/model_handler.c`

**Mục đích:** Implement callback để xử lý BACKPROP nhận được.

```c
/**
 * @brief Callback khi nhận BACKPROP_DATA
 *
 * Được gọi khi node này là đích cuối cùng của BACKPROP packet.
 *
 * @param srv   Gradient server instance
 * @param src   Địa chỉ node gửi gần nhất (không phải Gateway)
 * @param data  Dữ liệu payload
 */
static void handle_data_received(struct bt_mesh_gradient_srv *srv,
                                  uint16_t src,
                                  uint16_t data)
{
    LOG_INF("╔════════════════════════════════════════╗");
    LOG_INF("║     BACKPROP DATA RECEIVED!            ║");
    LOG_INF("╠════════════════════════════════════════╣");
    LOG_INF("║ From (last hop): 0x%04X                ", src);
    LOG_INF("║ Data: 0x%04X                           ", data);
    LOG_INF("╚════════════════════════════════════════╝");

    /*
     * Ứng dụng cụ thể có thể:
     * - Điều khiển LED dựa trên data
     * - Thay đổi cấu hình sensor
     * - Gửi acknowledgment ngược lại
     * - Trigger hành động cụ thể
     */

    /* Ví dụ: Blink LED khi nhận BACKPROP */
    led_indication_blink(LED_INDICATION_BACKPROP_RECEIVED);
}

/**
 * @brief Handlers cho gradient server
 */
static const struct bt_mesh_gradient_srv_handlers chat_handlers = {
    .start = handle_chat_start,
    .data_received = handle_data_received,  /* Callback MỚI */
};
```

---

#### 4.4. Ví Dụ Hoạt Động Chi Tiết

**Scenario: Gateway gửi BACKPROP đến Node C qua Node A và Node B**

```
Topology (đã học routes từ DATA uplink):
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  Gateway(0x0001)    NodeA(0x0002)    NodeB(0x0003)    NodeC(0x0004)│
│      g=0               g=1              g=2              g=3       │
│                                                                     │
│  RRT:                 RRT:              RRT:              RRT:      │
│  0x0004 via 0x0002    0x0004 via 0x0003 0x0004 via 0x0004 (empty)  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════
BƯỚC 1: Gateway gửi BACKPROP đến Node C
═══════════════════════════════════════════════════════════════════════

Gateway gọi:
  bt_mesh_gradient_srv_backprop_send(srv, 0x0004, 0xABCD)
                                          │        │
                                          │        └── payload
                                          └── destination (NodeC)

Tra cứu RRT:
  rrt_find_nexthop(table, 0x0004) → 0x0002 (NodeA)

Packet được tạo:
┌─────────────────────┬──────────────┬─────────────────┐
│ final_dest = 0x0004 │   ttl = 10   │ payload = 0xABCD│
└─────────────────────┴──────────────┴─────────────────┘

Gateway gửi packet đến NodeA (0x0002)

Log trên Gateway:
  [INF] BACKPROP send: dest=0x0004, data=0xABCD, nexthop=0x0002


═══════════════════════════════════════════════════════════════════════
BƯỚC 2: NodeA nhận và forward
═══════════════════════════════════════════════════════════════════════

NodeA's handle_backprop_message() được gọi:
  - ctx->addr = 0x0001 (sender = Gateway)
  - final_dest = 0x0004 (NodeC)
  - ttl = 10
  - payload = 0xABCD

Kiểm tra:
  - final_dest (0x0004) != my_addr (0x0002) → Không phải đích
  - ttl (10) > MIN_TTL (1) → Còn có thể forward

Tra cứu RRT:
  rrt_find_nexthop(table, 0x0004) → 0x0003 (NodeB)

Packet forward:
┌─────────────────────┬──────────────┬─────────────────┐
│ final_dest = 0x0004 │   ttl = 9    │ payload = 0xABCD│  ← TTL giảm!
└─────────────────────┴──────────────┴─────────────────┘

NodeA gửi đến NodeB (0x0003)

Log trên NodeA:
  [INF] BACKPROP received: sender=0x0001, dest=0x0004, ttl=10, data=0xABCD
  [DBG] BACKPROP forwarding: dest=0x0004 via nexthop=0x0003, new_ttl=9


═══════════════════════════════════════════════════════════════════════
BƯỚC 3: NodeB nhận và forward
═══════════════════════════════════════════════════════════════════════

NodeB's handle_backprop_message() được gọi:
  - ctx->addr = 0x0002 (sender = NodeA)
  - final_dest = 0x0004 (NodeC)
  - ttl = 9
  - payload = 0xABCD

Kiểm tra:
  - final_dest (0x0004) != my_addr (0x0003) → Không phải đích
  - ttl (9) > MIN_TTL (1) → Còn có thể forward

Tra cứu RRT:
  rrt_find_nexthop(table, 0x0004) → 0x0004 (NodeC - direct neighbor)

Packet forward:
┌─────────────────────┬──────────────┬─────────────────┐
│ final_dest = 0x0004 │   ttl = 8    │ payload = 0xABCD│  ← TTL giảm!
└─────────────────────┴──────────────┴─────────────────┘

NodeB gửi đến NodeC (0x0004)

Log trên NodeB:
  [INF] BACKPROP received: sender=0x0002, dest=0x0004, ttl=9, data=0xABCD
  [DBG] BACKPROP forwarding: dest=0x0004 via nexthop=0x0004, new_ttl=8


═══════════════════════════════════════════════════════════════════════
BƯỚC 4: NodeC nhận - ĐÍ C CUỐI CÙNG
═══════════════════════════════════════════════════════════════════════

NodeC's handle_backprop_message() được gọi:
  - ctx->addr = 0x0003 (sender = NodeB)
  - final_dest = 0x0004 (NodeC)
  - ttl = 8
  - payload = 0xABCD

Kiểm tra:
  - final_dest (0x0004) == my_addr (0x0004) → ĐÂY LÀ ĐÍCH!

Gọi callback:
  srv->handlers->data_received(srv, 0x0003, 0xABCD)

Log trên NodeC:
  [INF] BACKPROP received: sender=0x0003, dest=0x0004, ttl=8, data=0xABCD
  [INF] BACKPROP delivered! src=0x0003, data=0xABCD
  [INF] ╔════════════════════════════════════════╗
  [INF] ║     BACKPROP DATA RECEIVED!            ║
  [INF] ╠════════════════════════════════════════╣
  [INF] ║ From (last hop): 0x0003                
  [INF] ║ Data: 0xABCD                           
  [INF] ╚════════════════════════════════════════╝

THÀNH CÔNG! Dữ liệu từ Gateway đã đến NodeC!
```

---

#### 4.5. Xử Lý Các Trường Hợp Lỗi

##### 4.5.1. Không Có Route Đến Destination

```c
/*
 * Trường hợp: Gateway muốn gửi BACKPROP nhưng không có route
 */
int err = bt_mesh_gradient_srv_backprop_send(srv, 0x0004, 0x1234);
if (err == -ENOENT) {
    LOG_ERR("Cannot send BACKPROP: No route to 0x0004");
    /*
     * Giải pháp:
     * 1. Đợi node gửi DATA/heartbeat để học route
     * 2. Hoặc thông báo lỗi cho user
     */
}

/*
 * Trường hợp: Node trung gian không có route
 *
 * Xảy ra khi:
 * - Node trung gian restart và mất RRT
 * - Route expired trên node trung gian
 *
 * Packet sẽ bị drop tại node đó với log:
 * [WRN] BACKPROP: No route to destination 0x0004
 */
```

##### 4.5.2. TTL Expired

```c
/*
 * Trường hợp: TTL giảm đến MIN_TTL trước khi đến đích
 *
 * Có thể do:
 * 1. Loop trong mạng
 * 2. Destination quá xa (>10 hops với default TTL)
 *
 * Log trên node cuối:
 * [WRN] BACKPROP dropped: TTL expired (ttl=1, dest=0x0004)
 *
 * Giải pháp:
 * 1. Tăng DEFAULT_TTL nếu mạng lớn
 * 2. Kiểm tra topology để tìm loop
 */
```

##### 4.5.3. Packet Quá Ngắn

```c
/*
 * Trường hợp: Packet bị corrupted hoặc sai format
 *
 * Log:
 * [ERR] BACKPROP packet too short: 3 bytes (expected 5)
 *
 * Packet sẽ bị drop
 */
```

---

#### 4.6. Flow Diagram

```
                    ┌───────────────────────────────────────┐
                    │ bt_mesh_gradient_srv_backprop_send()  │
                    │ (Gateway gọi để gửi BACKPROP)         │
                    └───────────────────┬───────────────────┘
                                        │
                                        ▼
                    ┌───────────────────────────────────────┐
                    │    rrt_find_nexthop(dest)             │
                    └───────────────────┬───────────────────┘
                                        │
                         ┌──────────────┴──────────────┐
                         │                             │
                    nexthop found              nexthop not found
                         │                             │
                         ▼                             ▼
              ┌────────────────────┐        ┌────────────────────┐
              │ Tạo packet:        │        │ Return -ENOENT     │
              │ [dest, TTL, data]  │        │ (No route)         │
              │ Gửi đến nexthop    │        └────────────────────┘
              └─────────┬──────────┘
                        │
          ╔═════════════╧═════════════════════════════════════╗
          ║            NODE TRUNG GIAN NHẬN                   ║
          ╚═════════════╤═════════════════════════════════════╝
                        │
                        ▼
              ┌────────────────────┐
              │ Parse packet       │
              │ dest, ttl, payload │
              └─────────┬──────────┘
                        │
              ┌─────────┴─────────┐
              │                   │
         dest == my_addr     dest != my_addr
              │                   │
              ▼                   ▼
    ┌─────────────────┐  ┌─────────────────────┐
    │ ĐÍCH ĐÃ ĐẾN!    │  │ Kiểm tra TTL        │
    │ Gọi callback    │  └─────────┬───────────┘
    │ data_received() │            │
    └─────────────────┘   ┌────────┴────────┐
                          │                 │
                    ttl > MIN_TTL      ttl <= MIN_TTL
                          │                 │
                          ▼                 ▼
              ┌────────────────────┐ ┌────────────────────┐
              │ Tìm nexthop từ RRT │ │ DROP packet        │
              └─────────┬──────────┘ │ (TTL expired)      │
                        │            └────────────────────┘
                        │
         ┌──────────────┴──────────────┐
         │                             │
    nexthop found              nexthop not found
         │                             │
         ▼                             ▼
┌─────────────────────┐     ┌────────────────────┐
│ Forward packet với  │     │ DROP packet        │
│ TTL = TTL - 1       │     │ (No route)         │
│ Gửi đến nexthop     │     └────────────────────┘
└─────────────────────┘
```

---

#### 4.7. Tóm Tắt Thay Đổi

| File | Loại | Thay Đổi |
|------|------|----------|
| `include/gradient_srv.h` | Sửa | Thêm opcode `OP_BACKPROP_DATA` (0x0C) |
| `include/gradient_srv.h` | Sửa | Thêm constants: `DEFAULT_TTL`, `MIN_TTL`, `HEARTBEAT_MARKER` |
| `include/gradient_srv.h` | Sửa | Thêm callback type `data_received_t` |
| `include/gradient_srv.h` | Sửa | Thêm `data_received` vào struct handlers |
| `include/gradient_srv.h` | Sửa | Thêm prototype `bt_mesh_gradient_srv_backprop_send()` |
| `src/gradient_srv.c` | Sửa | Thêm `handle_backprop_message()` handler |
| `src/gradient_srv.c` | Sửa | Thêm entry vào opcode array |
| `src/gradient_srv.c` | Sửa | Implement `bt_mesh_gradient_srv_backprop_send()` |
| `src/model_handler.c` | Sửa | Thêm `handle_data_received()` callback |
| `src/model_handler.c` | Sửa | Thêm callback vào `chat_handlers` struct |

---

#### 4.8. Kết Quả

Sau khi implement giai đoạn 4:

✅ Có opcode mới cho BACKPROP_DATA (0x0C)  
✅ Packet format: [dest:2] + [ttl:1] + [payload:2] = 5 bytes  
✅ Handler xử lý: deliver locally hoặc forward  
✅ TTL mechanism chống loop  
✅ API để Gateway gửi BACKPROP: `bt_mesh_gradient_srv_backprop_send()`  
✅ Callback thông báo khi nhận BACKPROP tại destination  
✅ Xử lý các trường hợp lỗi (no route, TTL expired)  
✅ **GATEWAY CÓ THỂ GỬI DỮ LIỆU XUỐNG BẤT KỲ NODE NÀO!**

---

#### 4.9. Lưu Ý Quan Trọng

1. **TTL Value:**
   - Default 10 phù hợp với mạng <10 hops
   - Nếu mạng lớn hơn, tăng `DEFAULT_TTL`
   - Maximum TTL = 255 (uint8_t)

2. **Không Có ACK:**
   - BACKPROP không có acknowledgment
   - Gateway không biết packet có đến đích không
   - Nếu cần reliability, ứng dụng phải tự implement ACK

3. **Dependency:**
   - BACKPROP phụ thuộc vào RRT
   - RRT phụ thuộc vào DATA packets
   - Nếu node chưa gửi DATA, không thể gửi BACKPROP đến node đó

4. **Security:**
   - Không có authentication cho BACKPROP
   - Bất kỳ node nào cũng có thể giả mạo BACKPROP
   - Trong production, cần thêm security layer

---

### giai đoạn 5: Implement Heartbeat Mechanism

---

#### 5.1. Mục Tiêu

Implement cơ chế heartbeat để duy trì routes trong RRT, đảm bảo Gateway luôn có thể gửi BACKPROP đến các node ngay cả khi node không có dữ liệu thực để gửi.

---

#### 5.2. Phân Tích Vấn Đề

##### 5.2.1. Route Expiration Problem

**Scenario có vấn đề:**
```
Timeline:
T=0s      NodeC gửi DATA lên Gateway
          → Gateway học route đến NodeC
          
T=30s     Gateway có thể gửi BACKPROP đến NodeC ✓

T=60s     Gateway có thể gửi BACKPROP đến NodeC ✓

T=90s     Route đến NodeC EXPIRED và bị xóa!
          (RRT timeout = 90 giây)

T=100s    Gateway muốn gửi BACKPROP đến NodeC
          → "No route to 0x0004" ✗
          → THẤT BẠI!
```

**Nguyên nhân:**
- RRT entries có timeout (90 giây)
- Nếu node không gửi DATA trong thời gian dài, route bị xóa
- Gateway mất khả năng liên lạc với node

##### 5.2.2. Giải Pháp: Heartbeat

**Ý tưởng:**
- Mỗi node định kỳ gửi "heartbeat" - một DATA packet giả
- Heartbeat đi lên Gateway giống DATA thường
- Tất cả nodes trên đường đi refresh route
- Route không bao giờ expired (nếu node còn sống)

**Scenario với heartbeat:**
```
Timeline:
T=0s      NodeC gửi DATA lên Gateway
          → Gateway học route đến NodeC

T=30s     NodeC gửi HEARTBEAT [src=C, data=0xFFFF]
          → Route được refresh
          
T=60s     NodeC gửi HEARTBEAT
          → Route được refresh

T=90s     NodeC gửi HEARTBEAT
          → Route được refresh
          → Route KHÔNG bị xóa!

T=100s    Gateway gửi BACKPROP đến NodeC
          → Route vẫn còn, gửi thành công ✓
```

---

#### 5.3. Thiết Kế Heartbeat

##### 5.3.1. Heartbeat Packet Format

```
Heartbeat sử dụng cùng format với DATA packet:
┌─────────────────────┬─────────────────────┐
│ original_source (2B)│ data = 0xFFFF (2B)  │ ← HEARTBEAT_MARKER
└─────────────────────┴─────────────────────┘

Đặc điểm:
- Cùng opcode với DATA (0x0A)
- Cùng format (4 bytes)
- Khác nhau ở payload: 0xFFFF = heartbeat marker
- Được route và học giống hệt DATA thường
```

##### 5.3.2. Tại Sao Dùng 0xFFFF Làm Marker?

```
Lý do chọn 0xFFFF:
1. Giá trị lớn nhất của uint16_t
2. Ít khả năng trùng với data thực từ sensor
3. Dễ nhận biết trong log/debug
4. Một giá trị duy nhất, không cần thêm field

So sánh:
┌─────────────────────────────────────────────────────────────┐
│ Phương án     │ Ưu điểm           │ Nhược điểm              │
├───────────────┼───────────────────┼─────────────────────────┤
│ Dùng 0xFFFF   │ Không thay đổi    │ Mất 1 giá trị data      │
│ (chọn)        │ packet format     │ (hiếm khi cần 0xFFFF)   │
├───────────────┼───────────────────┼─────────────────────────┤
│ Thêm flag     │ Rõ ràng hơn       │ Thay đổi packet format  │
│ trong packet  │                   │ Không backward compat   │
├───────────────┼───────────────────┼─────────────────────────┤
│ Opcode riêng  │ Tách biệt hoàn    │ Phức tạp hơn            │
│               │ toàn              │ Duplicate logic         │
└───────────────┴───────────────────┴─────────────────────────┘
```

##### 5.3.3. Heartbeat Timing

```
┌─────────────────────────────────────────────────────────────────────┐
│                    HEARTBEAT TIMING DIAGRAM                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Node khởi động                                                     │
│       │                                                             │
│       ▼                                                             │
│  ┌─────────────┐                                                    │
│  │ Random delay│  0-10 giây (tránh collision)                       │
│  │ (initial)   │                                                    │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │ Gửi HB #1   │                                                    │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         │  30 giây (HEARTBEAT_INTERVAL_SEC)                         │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │ Gửi HB #2   │                                                    │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         │  30 giây                                                  │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │ Gửi HB #3   │  ... tiếp tục mãi mãi                              │
│  └─────────────┘                                                    │
│                                                                     │
│  Route timeout = 90 giây = 3 × 30 giây                              │
│  → Cho phép miss 2 heartbeat trước khi route bị xóa                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

##### 5.3.4. Ai Gửi Heartbeat?

```
┌─────────────────────────────────────────────────────────────────────┐
│ Node Type      │ Gửi Heartbeat? │ Lý do                             │
├────────────────┼────────────────┼───────────────────────────────────┤
│ Gateway        │      KHÔNG     │ Gateway là đích của DATA,         │
│ (gradient=0)   │                │ không cần gửi gì lên              │
├────────────────┼────────────────┼───────────────────────────────────┤
│ Regular Node   │       CÓ       │ Cần duy trì route để Gateway      │
│ (gradient>0)   │                │ có thể gửi BACKPROP xuống         │
├────────────────┼────────────────┼───────────────────────────────────┤
│ Unprovisioned  │      KHÔNG     │ gradient=255 (chưa set)           │
│ Node           │                │ Không có trong mesh               │
└────────────────┴────────────────┴───────────────────────────────────┘
```

---

#### 5.4. Chi Tiết Thay Đổi Code

##### 5.4.1. File `Kconfig`

**Mục đích:** Thêm config options cho heartbeat.

```kconfig
menu "Gradient Server Heartbeat"

config BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    bool "Enable heartbeat mechanism"
    default y
    help
      Khi enable, nodes sẽ định kỳ gửi heartbeat packets
      để duy trì reverse routes trong RRT.
      
      Gateway (sink node) không gửi heartbeat.
      
      Nếu disable, routes có thể expired nếu node không
      gửi DATA trong thời gian dài.

config BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC
    int "Heartbeat interval in seconds"
    default 30
    range 10 300
    depends on BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED
    help
      Khoảng thời gian giữa các heartbeat packets (giây).
      
      Giá trị nhỏ hơn: Routes fresh hơn, nhưng tăng traffic
      Giá trị lớn hơn: Ít traffic, nhưng routes có thể stale
      
      Khuyến nghị: 30 giây (default)
      RRT timeout nên = 3 × heartbeat interval

endmenu
```

---

##### 5.4.2. File `include/heartbeat.h` (FILE MỚI)

**Mục đích:** Định nghĩa API cho heartbeat module.

```c
/**
 * @file heartbeat.h
 * @brief Heartbeat mechanism cho Backpropagation Routing
 *
 * Module này quản lý việc gửi heartbeat packets định kỳ
 * để duy trì reverse routes trong RRT.
 *
 * Heartbeat là DATA packet với payload = 0xFFFF (HEARTBEAT_MARKER)
 */

#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_

#include <zephyr/kernel.h>
#include "gradient_srv.h"

/**
 * @brief Khởi tạo heartbeat module
 *
 * @param srv  Con trỏ đến gradient server instance
 *
 * Khởi tạo work queue và các biến nội bộ.
 * Phải được gọi trước khi sử dụng các hàm khác.
 */
void heartbeat_init(struct bt_mesh_gradient_srv *srv);

/**
 * @brief Bắt đầu gửi heartbeat
 *
 * Bắt đầu schedule heartbeat với random initial delay.
 * 
 * Heartbeat chỉ được gửi nếu:
 * - Heartbeat enabled trong Kconfig
 * - Node không phải Gateway (gradient > 0)
 * - Node đã được provisioned (gradient != 255)
 *
 * Thường được gọi sau khi mesh provisioned và ready.
 */
void heartbeat_start(void);

/**
 * @brief Dừng gửi heartbeat
 *
 * Cancel pending heartbeat work.
 * Heartbeat sẽ không được gửi cho đến khi gọi start() lại.
 */
void heartbeat_stop(void);

/**
 * @brief Cập nhật khi gradient thay đổi
 *
 * @param new_gradient  Giá trị gradient mới
 *
 * Được gọi mỗi khi node nhận GRADIENT message và cập nhật gradient.
 *
 * Logic:
 * - Nếu gradient mới = 0 (trở thành Gateway): Stop heartbeat
 * - Nếu gradient mới > 0 và chưa start: Start heartbeat
 * - Nếu đã running: Không làm gì
 */
void heartbeat_update_gradient(uint8_t new_gradient);

/**
 * @brief Kiểm tra heartbeat đang active không
 *
 * @return true nếu heartbeat đang được schedule
 * @return false nếu heartbeat đã stop hoặc chưa start
 */
bool heartbeat_is_active(void);

/**
 * @brief Lấy thời gian đến heartbeat tiếp theo (debug)
 *
 * @return Số milliseconds đến heartbeat tiếp theo
 *         0 nếu heartbeat không active
 */
int64_t heartbeat_time_to_next(void);

#endif /* HEARTBEAT_H_ */
```

---

##### 5.4.3. File `src/heartbeat.c` (FILE MỚI)

**Mục đích:** Implement heartbeat logic.

```c
/**
 * @file heartbeat.c
 * @brief Implementation của Heartbeat mechanism
 */

#include "heartbeat.h"
#include "data_forward.h"
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(heartbeat, CONFIG_BT_MESH_MODEL_LOG_LEVEL);

/*============================================================
 * Configuration
 *============================================================*/

/** Heartbeat interval từ Kconfig (milliseconds) */
#define HEARTBEAT_INTERVAL_MS \
    (CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC * 1000)

/** Maximum initial delay để tránh collision (milliseconds) */
#define HEARTBEAT_INITIAL_DELAY_MAX_MS  10000

/*============================================================
 * Private Variables
 *============================================================*/

/** Reference đến gradient server */
static struct bt_mesh_gradient_srv *heartbeat_srv;

/** Delayable work cho heartbeat scheduling */
static struct k_work_delayable heartbeat_work;

/** Flag đánh dấu heartbeat đang active */
static bool heartbeat_active;

/** Gradient hiện tại của node */
static uint8_t current_gradient = 255;  /* 255 = unprovisioned */

/*============================================================
 * Private Functions
 *============================================================*/

/**
 * @brief Kiểm tra xem có nên gửi heartbeat không
 *
 * @return true nếu nên gửi
 */
static bool should_send_heartbeat(void)
{
    /* Không gửi nếu heartbeat disabled */
    if (!IS_ENABLED(CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED)) {
        return false;
    }

    /* Không gửi nếu chưa có server reference */
    if (!heartbeat_srv) {
        return false;
    }

    /* Không gửi nếu là Gateway (gradient = 0) */
    if (current_gradient == 0) {
        LOG_DBG("Heartbeat: Skipping - this is Gateway");
        return false;
    }

    /* Không gửi nếu chưa provisioned (gradient = 255) */
    if (current_gradient == 255) {
        LOG_DBG("Heartbeat: Skipping - not provisioned yet");
        return false;
    }

    return true;
}

/**
 * @brief Work handler - thực hiện gửi heartbeat
 *
 * @param work  Work item (không sử dụng)
 */
static void heartbeat_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!should_send_heartbeat()) {
        /* Điều kiện không còn phù hợp, stop heartbeat */
        heartbeat_active = false;
        return;
    }

    /*
     * Gửi heartbeat packet
     *
     * Sử dụng data_forward_send_direct() với payload = HEARTBEAT_MARKER
     * Packet sẽ được route lên Gateway giống DATA thường
     */
    LOG_INF("Heartbeat: Sending (gradient=%d)", current_gradient);

    int err = data_forward_send_direct(heartbeat_srv,
                                       BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER);
    if (err) {
        LOG_WRN("Heartbeat: Failed to send (err=%d)", err);
    }

    /*
     * Schedule heartbeat tiếp theo
     */
    k_work_schedule(&heartbeat_work, K_MSEC(HEARTBEAT_INTERVAL_MS));

    LOG_DBG("Heartbeat: Next in %d seconds",
            CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC);
}

/*============================================================
 * Public Functions
 *============================================================*/

void heartbeat_init(struct bt_mesh_gradient_srv *srv)
{
    heartbeat_srv = srv;
    heartbeat_active = false;
    current_gradient = 255;

    /* Khởi tạo delayable work */
    k_work_init_delayable(&heartbeat_work, heartbeat_work_handler);

    LOG_INF("Heartbeat: Initialized (interval=%d sec)",
            CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC);
}

void heartbeat_start(void)
{
    if (!IS_ENABLED(CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED)) {
        LOG_INF("Heartbeat: Disabled in Kconfig");
        return;
    }

    if (heartbeat_active) {
        LOG_DBG("Heartbeat: Already active");
        return;
    }

    if (!should_send_heartbeat()) {
        LOG_INF("Heartbeat: Conditions not met, not starting");
        return;
    }

    /*
     * Random initial delay để tránh collision
     *
     * Khi nhiều nodes khởi động cùng lúc (ví dụ sau power outage),
     * nếu tất cả gửi heartbeat cùng lúc sẽ gây collision.
     * Random delay 0-10 giây giúp spread out các heartbeats.
     */
    uint32_t initial_delay_ms = sys_rand32_get() % HEARTBEAT_INITIAL_DELAY_MAX_MS;

    LOG_INF("Heartbeat: Starting with initial delay %d ms", initial_delay_ms);

    heartbeat_active = true;
    k_work_schedule(&heartbeat_work, K_MSEC(initial_delay_ms));
}

void heartbeat_stop(void)
{
    if (!heartbeat_active) {
        return;
    }

    LOG_INF("Heartbeat: Stopping");

    heartbeat_active = false;
    k_work_cancel_delayable(&heartbeat_work);
}

void heartbeat_update_gradient(uint8_t new_gradient)
{
    uint8_t old_gradient = current_gradient;
    current_gradient = new_gradient;

    LOG_DBG("Heartbeat: Gradient updated %d -> %d", old_gradient, new_gradient);

    /*
     * Xử lý các trường hợp chuyển đổi
     */

    if (new_gradient == 0) {
        /*
         * Trở thành Gateway - stop heartbeat
         *
         * Gateway không cần gửi heartbeat vì nó là đích
         * của tất cả DATA packets
         */
        if (heartbeat_active) {
            LOG_INF("Heartbeat: Stopping (became Gateway)");
            heartbeat_stop();
        }
        return;
    }

    if (old_gradient == 255 && new_gradient > 0 && new_gradient < 255) {
        /*
         * Node mới được provisioned và có gradient hợp lệ
         * Start heartbeat nếu chưa active
         */
        if (!heartbeat_active) {
            LOG_INF("Heartbeat: Starting (node provisioned with gradient=%d)",
                    new_gradient);
            heartbeat_start();
        }
        return;
    }

    /* Gradient thay đổi bình thường (vẫn > 0), không làm gì */
}

bool heartbeat_is_active(void)
{
    return heartbeat_active;
}

int64_t heartbeat_time_to_next(void)
{
    if (!heartbeat_active) {
        return 0;
    }

    return k_work_delayable_remaining_get(&heartbeat_work);
}
```

---

##### 5.4.4. File `src/model_handler.c`

**Mục đích:** Tích hợp heartbeat vào model handler.

**Thêm include:**

```c
#include "heartbeat.h"
```

**Sửa hàm init:**

```c
/**
 * @brief Khởi tạo model handler
 */
void model_handler_init(void)
{
    /* ... existing init code ... */

    /* Khởi tạo Reverse Routing Table */
    rrt_init(gradient_srv.fwd_table);

    /* Khởi tạo Heartbeat module */
    heartbeat_init(&gradient_srv);

    LOG_INF("Model handler initialized");
}
```

**Sửa callback start:**

```c
/**
 * @brief Callback khi mesh provisioned và ready
 *
 * Được gọi khi BLE Mesh stack đã sẵn sàng hoạt động.
 */
static void handle_chat_start(struct bt_mesh_gradient_srv *srv)
{
    LOG_INF("Mesh ready, starting services");

    /*
     * Bắt đầu heartbeat
     *
     * Heartbeat module sẽ tự kiểm tra:
     * - Có phải Gateway không
     * - Gradient đã được set chưa
     */
    heartbeat_start();
}
```

---

##### 5.4.5. File `src/gradient_work.c`

**Mục đích:** Cập nhật heartbeat khi gradient thay đổi.

**Thêm include:**

```c
#include "heartbeat.h"
```

**Sửa nơi gradient được cập nhật:**

```c
/**
 * @brief Cập nhật gradient từ GRADIENT message nhận được
 *
 * @param srv       Gradient server
 * @param gradient  Giá trị gradient mới
 */
void gradient_work_update(struct bt_mesh_gradient_srv *srv, uint8_t gradient)
{
    uint8_t old_gradient = srv->gradient;

    /* Cập nhật gradient */
    srv->gradient = gradient;

    LOG_INF("Gradient updated: %d -> %d", old_gradient, gradient);

    /*
     * Thông báo cho heartbeat module
     *
     * Heartbeat cần biết gradient để:
     * - Stop nếu trở thành Gateway (gradient=0)
     * - Start nếu mới được provisioned
     */
    heartbeat_update_gradient(gradient);

    /* ... rest of update logic ... */
}
```

---

##### 5.4.6. File `src/gradient_srv.c`

**Mục đích:** Phân biệt heartbeat và data thực khi log.

```c
/**
 * @brief Handler cho DATA message
 */
static int handle_data_message(const struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
    /* ... parse packet ... */

    uint16_t original_source = net_buf_simple_pull_le16(buf);
    uint16_t data = net_buf_simple_pull_le16(buf);

    /*
     * Phân biệt heartbeat và data thực
     */
    bool is_heartbeat = (data == BT_MESH_GRADIENT_SRV_HEARTBEAT_MARKER);

    if (is_heartbeat) {
        LOG_DBG("Heartbeat received from 0x%04X (via 0x%04X)",
                original_source, ctx->addr);
    } else {
        LOG_INF("DATA received: src=0x%04X, via=0x%04X, data=0x%04X",
                original_source, ctx->addr, data);
    }

    /*
     * Route learning: Giống nhau cho cả heartbeat và data thực
     *
     * Heartbeat mục đích chính là refresh routes!
     */
    rrt_add_dest(srv->fwd_table, ctx->addr, original_source, k_uptime_get());

    /* ... rest of handler ... */

    /*
     * Tại Gateway: Xử lý khác nhau
     */
    if (srv->gradient == 0) {
        if (is_heartbeat) {
            LOG_DBG("Gateway: Heartbeat from node 0x%04X", original_source);
            /* Không cần xử lý thêm cho heartbeat */
        } else {
            LOG_INF("Gateway: Data from node 0x%04X: 0x%04X",
                    original_source, data);
            /* Có thể gọi callback hoặc xử lý data */
        }
        return 0;
    }

    /* Forward packet lên nexthop */
    return data_forward_send(srv, data, original_source);
}
```

---

##### 5.4.7. File `CMakeLists.txt`

**Mục đích:** Thêm source file heartbeat.c.

```cmake
target_sources(app PRIVATE
    src/main.c
    src/model_handler.c
    src/gradient_srv.c
    src/neighbor_table.c
    src/routing_policy.c
    src/led_indication.c
    src/data_forward.c
    src/gradient_work.c
    src/reverse_routing.c
    src/heartbeat.c           # <-- THÊM DÒNG NÀY
)
```

---

#### 5.5. Ví Dụ Hoạt Động Chi Tiết

**Scenario: Node C duy trì route với heartbeat**

```
Topology:
Gateway(0x0001) ←── NodeA(0x0002) ←── NodeB(0x0003) ←── NodeC(0x0004)
   g=0                 g=1                g=2                g=3


═══════════════════════════════════════════════════════════════════════
PHASE 1: Khởi động và Initial Heartbeat
═══════════════════════════════════════════════════════════════════════

T=0s     Tất cả nodes khởi động, được provisioned

         NodeC: heartbeat_start()
                → Random delay = 3.5 giây

         NodeB: heartbeat_start()
                → Random delay = 7.2 giây

         NodeA: heartbeat_start()
                → Random delay = 1.8 giây

T=1.8s   NodeA gửi Heartbeat #1
         [original_src=0x0002, data=0xFFFF]
         → Gateway học route đến NodeA

T=3.5s   NodeC gửi Heartbeat #1
         [original_src=0x0004, data=0xFFFF]
         → NodeB học route đến NodeC
         → NodeA học route đến NodeC
         → Gateway học route đến NodeC

T=7.2s   NodeB gửi Heartbeat #1
         [original_src=0x0003, data=0xFFFF]
         → NodeA học route đến NodeB
         → Gateway học route đến NodeB


═══════════════════════════════════════════════════════════════════════
PHASE 2: Regular Heartbeats (mỗi 30 giây)
═══════════════════════════════════════════════════════════════════════

T=31.8s  NodeA gửi Heartbeat #2
T=33.5s  NodeC gửi Heartbeat #2
         → Tất cả routes đến NodeC được refresh
         → last_seen = T=33.5s

T=37.2s  NodeB gửi Heartbeat #2

T=61.8s  NodeA gửi Heartbeat #3
T=63.5s  NodeC gửi Heartbeat #3
         → Routes đến NodeC refresh lại
         → last_seen = T=63.5s

...tiếp tục mãi mãi...


═══════════════════════════════════════════════════════════════════════
PHASE 3: Route Timeout Avoidance
═══════════════════════════════════════════════════════════════════════

RRT Timeout = 90 giây
Heartbeat Interval = 30 giây

Timeline cho route đến NodeC tại Gateway:
┌────────┬───────────────┬─────────────────────────────────────┐
│ Time   │ Event         │ Route Age (giây)                    │
├────────┼───────────────┼─────────────────────────────────────┤
│ 3.5s   │ HB #1         │ 0 (mới học)                         │
│ 33.5s  │ HB #2         │ 0 (refresh)                         │
│ 63.5s  │ HB #3         │ 0 (refresh)                         │
│ 93.5s  │ HB #4         │ 0 (refresh)                         │
│ ...    │ ...           │ ...                                 │
└────────┴───────────────┴─────────────────────────────────────┘

Route KHÔNG BAO GIỜ đạt 90 giây (timeout)
→ Route luôn tồn tại!


═══════════════════════════════════════════════════════════════════════
PHASE 4: Gateway Gửi BACKPROP Bất Kỳ Lúc Nào
═══════════════════════════════════════════════════════════════════════

T=100s   Gateway muốn gửi BACKPROP đến NodeC

         Kiểm tra RRT:
         - Route đến 0x0004: VIA 0x0002, last_seen=93.5s, age=6.5s
         - Route còn valid ✓

         bt_mesh_gradient_srv_backprop_send(srv, 0x0004, 0xABCD)
         → THÀNH CÔNG!
```

---

#### 5.6. Xử Lý Các Trường Hợp Đặc Biệt

##### 5.6.1. Node Restart

```
Scenario: NodeC restart giữa chừng

T=50s    NodeC đang gửi heartbeat bình thường

T=55s    NodeC RESTART!
         - Mất state
         - heartbeat_active = false
         - current_gradient = 255

T=56s    NodeC khởi động xong
         - heartbeat_init() được gọi
         - Nhưng gradient chưa được set (= 255)
         - heartbeat_start() → skip (not provisioned yet)

T=60s    NodeC nhận GRADIENT message
         - current_gradient = 3
         - heartbeat_update_gradient(3)
         - Vì old=255, new=3 → heartbeat_start()

T=60s + random_delay
         NodeC gửi Heartbeat
         → Routes được refresh
         → Hệ thống hoạt động bình thường

Trong khoảng T=55s đến T=60s+delay:
- Routes vẫn còn valid (chưa đến 90s timeout)
- Gateway vẫn có thể gửi BACKPROP đến NodeC
```

##### 5.6.2. Node Tắt Hoàn Toàn

```
Scenario: NodeC bị tắt nguồn và không bật lại

T=63.5s  NodeC gửi Heartbeat cuối cùng

T=70s    NodeC bị TẮT NGUỒN

T=93.5s  Heartbeat expected nhưng KHÔNG ĐẾN
         - Routes vẫn còn valid (age = 30s < 90s)

T=123.5s Heartbeat expected nhưng KHÔNG ĐẾN
         - Routes vẫn còn valid (age = 60s < 90s)

T=153.5s RRT Cleanup chạy (mỗi 15 giây)
         - Route đến NodeC: age = 90s = timeout
         - Route bị XÓA

T=160s   Gateway muốn gửi BACKPROP đến NodeC
         → "No route to 0x0004"
         → Đúng! NodeC đã offline

T=200s   NodeC được BẬT LẠI
         → Sau vài giây, gửi Heartbeat
         → Route được học lại
         → Gateway có thể gửi BACKPROP
```

##### 5.6.3. Gateway Restart

```
Scenario: Gateway restart

Gateway KHÔNG gửi heartbeat (gradient = 0)
Khi Gateway restart:
- RRT bị mất
- Cần đợi nodes gửi heartbeat để học lại routes

Timeline:
T=0s     Gateway restart, RRT rỗng

T=5s     NodeC gửi heartbeat
         → Gateway học route đến NodeC

T=15s    NodeA gửi heartbeat
         → Gateway học route đến NodeA

T=30s    Gateway đã có routes đến tất cả nodes
         → Có thể gửi BACKPROP bình thường
```

---

#### 5.7. Flow Diagram

```
                    ┌───────────────────────────────────────┐
                    │          heartbeat_init()             │
                    │  (Gọi khi khởi động)                  │
                    └───────────────────┬───────────────────┘
                                        │
                                        ▼
                    ┌───────────────────────────────────────┐
                    │          heartbeat_start()            │
                    │  (Gọi khi mesh ready)                 │
                    └───────────────────┬───────────────────┘
                                        │
                         ┌──────────────┴──────────────┐
                         │                             │
                  should_send?                   should NOT send
                         │                             │
                    ┌────┴────┐                        ▼
                    │         │               ┌────────────────┐
              gradient=0  gradient>0          │ Return (skip)  │
              (Gateway)   (Regular)           └────────────────┘
                    │         │
                    ▼         ▼
           ┌──────────┐  ┌──────────────────────────────┐
           │ Stop     │  │ Schedule với random delay    │
           │ heartbeat│  │ (0-10 giây)                  │
           └──────────┘  └──────────────┬───────────────┘
                                        │
          ╔═════════════════════════════╧═════════════════════════╗
          ║              HEARTBEAT WORK HANDLER                   ║
          ╚═════════════════════════════╤═════════════════════════╝
                                        │
                                        ▼
                    ┌───────────────────────────────────────┐
                    │  data_forward_send_direct()           │
                    │  với data = 0xFFFF (HEARTBEAT_MARKER) │
                    └───────────────────┬───────────────────┘
                                        │
                                        ▼
                    ┌───────────────────────────────────────┐
                    │  Schedule heartbeat tiếp theo         │
                    │  sau HEARTBEAT_INTERVAL_SEC giây      │
                    └───────────────────┬───────────────────┘
                                        │
                                        │
                         ╔══════════════╧══════════════╗
                         ║  Loop mãi mãi cho đến khi   ║
                         ║  heartbeat_stop() được gọi  ║
                         ╚═════════════════════════════╝
```

---

#### 5.8. Tóm Tắt Thay Đổi

| File | Loại | Thay Đổi |
|------|------|----------|
| `Kconfig` | Sửa | Thêm `HEARTBEAT_ENABLED` option |
| `Kconfig` | Sửa | Thêm `HEARTBEAT_INTERVAL_SEC` option |
| `include/heartbeat.h` | **MỚI** | API cho heartbeat module |
| `src/heartbeat.c` | **MỚI** | Implementation heartbeat |
| `src/model_handler.c` | Sửa | Gọi `heartbeat_init()` và `heartbeat_start()` |
| `src/gradient_work.c` | Sửa | Gọi `heartbeat_update_gradient()` |
| `src/gradient_srv.c` | Sửa | Phân biệt heartbeat và data trong log |
| `CMakeLists.txt` | Sửa | Thêm `src/heartbeat.c` |

---

#### 5.9. Kết Quả

Sau khi implement giai đoạn 5:

✅ Heartbeat module với API đầy đủ  
✅ Nodes gửi heartbeat mỗi 30 giây (configurable)  
✅ Random initial delay tránh collision  
✅ Gateway không gửi heartbeat  
✅ Auto-stop khi trở thành Gateway  
✅ Auto-start khi được provisioned  
✅ Routes được refresh định kỳ  
✅ **ROUTES KHÔNG BAO GIỜ EXPIRED (nếu node còn sống)!**

---

#### 5.10. Lưu Ý Quan Trọng

1. **Interval vs Timeout:**
   ```
   RRT_TIMEOUT = 3 × HEARTBEAT_INTERVAL
   
   Với default:
   - Heartbeat: 30 giây
   - Timeout: 90 giây
   
   → Cho phép miss 2 heartbeat trước khi route bị xóa
   → Node có thể tạm thời unreachable mà không mất route
   ```

2. **Network Traffic:**
   ```
   Với N nodes, mỗi 30 giây có N-1 heartbeat packets
   
   Ví dụ: 10 nodes
   - 9 heartbeats × 4 bytes = 36 bytes mỗi 30 giây
   - ~1.2 bytes/giây
   - Rất nhỏ so với bandwidth BLE Mesh
   ```

3. **Random Delay Quan Trọng:**
   - Tránh "thundering herd" khi nhiều nodes khởi động cùng lúc
   - Spread out heartbeats giúp giảm collision
   - Delay 0-10 giây đủ cho hầu hết scenarios

4. **Heartbeat vs Real Data:**
   - Heartbeat và DATA có cùng effect: refresh routes
   - Nếu node gửi DATA thường xuyên, heartbeat vẫn gửi
   - Không có logic "skip heartbeat if sent data recently"
   - Trade-off: đơn giản hơn, nhưng có thể redundant

---

### giai đoạn 6: Cleanup và Integration

---

#### 6.1. Mục Tiêu

Hoàn thiện hệ thống Backpropagation Routing với:
1. **Cleanup tự động**: Xóa các route expired trong RRT
2. **Testing interface**: Button handlers để test từ hardware
3. **Configuration**: Các config options cần thiết
4. **Board support**: Config cho các board variants

---

#### 6.2. Phân Tích Yêu Cầu

##### 6.2.1. Tại Sao Cần Cleanup?

```
Vấn đề: RRT có thể chứa routes cũ/invalid

Scenario:
T=0s     NodeC gửi heartbeat
         → Gateway học route đến NodeC

T=30s    NodeC gửi heartbeat
         → Route refresh

T=35s    NodeC BỊ TẮT hoặc ra khỏi mạng

T=60s    Heartbeat expected nhưng KHÔNG ĐẾN
T=90s    Heartbeat expected nhưng KHÔNG ĐẾN
T=120s   Route đến NodeC đã "stale" 85 giây

Nếu không có cleanup:
- Route vẫn còn trong RRT
- Gateway gửi BACKPROP đến NodeC
- Packet bị mất vì NodeC không còn
- Không có error feedback

Với cleanup:
- Route bị xóa sau 90 giây không refresh
- Gateway biết không có route đến NodeC
- Có thể thông báo lỗi cho user
```

##### 6.2.2. Testing Interface Requirements

```
┌─────────────────────────────────────────────────────────────────────┐
│                    TESTING REQUIREMENTS                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  Button 1: Action                                                   │
│  ─────────                                                          │
│  • Trên Gateway: Gửi BACKPROP đến node đầu tiên trong RRT          │
│  • Trên Node: Gửi DATA lên Gateway (như trước)                      │
│                                                                     │
│  Button 2: Debug Info                                               │
│  ────────────────────                                               │
│  • In Forwarding Table                                              │
│  • In Reverse Routing Table                                         │
│  • In Heartbeat status                                              │
│  • In Node info (address, gradient)                                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

#### 6.3. Chi Tiết Thay Đổi Code

##### 6.3.1. File `Kconfig`

**Mục đích:** Thêm config options cho RRT timeout và capacity.

```kconfig
config BT_MESH_GRADIENT_SRV_RRT_TIMEOUT_SEC
    int "RRT entry timeout in seconds"
    default 90
    range 30 600
    help
      Thời gian timeout cho entries trong Reverse Routing Table.
      
      Khi một route không được refresh (nhận DATA/heartbeat)
      trong khoảng thời gian này, nó sẽ bị xóa.
      
      Khuyến nghị: 3 × HEARTBEAT_INTERVAL_SEC
      Cho phép miss 2 heartbeat trước khi xóa route.
      
      Giá trị mặc định: 90 giây (3 × 30 giây heartbeat)

config BT_MESH_GRADIENT_SRV_RRT_MAX_DEST
    int "Max reverse routes per neighbor"
    default 50
    range 5 500
    help
      Maximum number of destination nodes that can be stored in the 
      Reverse Routing Table for a single immediate neighbor.
      
      For the Gateway (Sink Node), this value should be at least equal 
      to the total number of nodes in the network, as one neighbor 
      might be the next-hop for all other nodes.
      
      Impact: Higher values consume more heap memory (approx 16 bytes per entry).
```

---

##### 6.3.2. File `include/reverse_routing.h`

**Mục đích:** Thêm helper function cho testing.

```c
/**
 * @brief Lấy một destination bất kỳ từ RRT
 *
 * @param table     Con trỏ đến forwarding table
 * @param out_dest  [out] Địa chỉ destination tìm được
 *
 * @return 0 nếu tìm thấy, -ENOENT nếu RRT rỗng
 *
 * Hàm helper để test - lấy destination đầu tiên trong RRT.
 * Được sử dụng bởi button handler trên Gateway để gửi BACKPROP
 * đến một node bất kỳ khi không biết node nào available.
 */
int rrt_get_any_destination(neighbor_entry_t *table, uint16_t *out_dest);
```

---

##### 6.3.3. File `src/reverse_routing.c`

**Mục đích:** Implement helper function và đảm bảo cleanup hoạt động.

**Hàm `rrt_get_any_destination()`:**

```c
/**
 * @brief Lấy destination đầu tiên trong RRT
 *
 * Duyệt qua tất cả neighbors trong forwarding table,
 * trả về destination đầu tiên tìm thấy.
 *
 * Useful cho testing khi muốn gửi BACKPROP nhưng không
 * biết cụ thể node nào available.
 */
int rrt_get_any_destination(neighbor_entry_t *table, uint16_t *out_dest)
{
    if (!table || !out_dest) {
        return -EINVAL;
    }

    /* Duyệt tất cả neighbors */
    for (int i = 0; i < FWD_TABLE_SIZE; i++) {
        if (table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            continue;  /* Entry trống */
        }

        /* Kiểm tra neighbor có destinations không */
        if (table[i].backprop_dest != NULL) {
            *out_dest = table[i].backprop_dest->addr;
            LOG_DBG("RRT: Found destination 0x%04X via 0x%04X",
                    *out_dest, table[i].addr);
            return 0;
        }
    }

    LOG_WRN("RRT: No destinations found (table empty)");
    return -ENOENT;
}
```

**Hàm `rrt_cleanup_expired()` (đã implement từ giai đoạn 2, review lại):**

```c
/**
 * @brief Xóa các entry hết hạn trong RRT
 *
 * @param table       Con trỏ đến forwarding table
 * @param timeout_ms  Thời gian timeout (milliseconds)
 *
 * Duyệt qua tất cả destinations trong RRT.
 * Xóa những entry có (current_time - last_seen) > timeout_ms.
 */
void rrt_cleanup_expired(neighbor_entry_t *table, int64_t timeout_ms)
{
    if (!table) {
        return;
    }

    int64_t now = k_uptime_get();
    int removed_count = 0;

    /* Duyệt tất cả neighbors */
    for (int i = 0; i < FWD_TABLE_SIZE; i++) {
        if (table[i].addr == BT_MESH_ADDR_UNASSIGNED) {
            continue;
        }

        /* Duyệt linked list của neighbor này */
        backprop_node_t *prev = NULL;
        backprop_node_t *curr = table[i].backprop_dest;

        while (curr) {
            backprop_node_t *next = curr->next;
            int64_t age = now - curr->last_seen;

            /* Kiểm tra expired */
            if (age > timeout_ms) {
                LOG_INF("RRT cleanup: Removing dest 0x%04X (age: %lld ms, timeout: %lld ms)",
                        curr->addr, age, timeout_ms);

                /* Xóa node này */
                if (prev) {
                    prev->next = next;
                } else {
                    table[i].backprop_dest = next;
                }

                /* Giải phóng memory */
                k_free(curr);
                removed_count++;
            } else {
                prev = curr;
            }

            curr = next;
        }
    }

    if (removed_count > 0) {
        LOG_INF("RRT cleanup: Removed %d expired entries", removed_count);
    }
}
```

---

##### 6.3.4. File `src/gradient_work.c`

**Mục đích:** Tích hợp RRT cleanup vào periodic cleanup routine.

**Thêm include:**

```c
#include "reverse_routing.h"
```

**Sửa cleanup handler:**

```c
/**
 * @brief Handler cho periodic cleanup
 *
 * Chạy mỗi CLEANUP_INTERVAL (15 giây) để:
 * 1. Xóa neighbors expired trong Forwarding Table
 * 2. Xóa routes expired trong RRT
 * 3. Cập nhật gradient dựa trên best parent còn lại
 * 4. Deferred publication để tránh deadlock
 */
static void cleanup_handler(struct k_work *work)
{
    if (g_gradient_srv == NULL) {
        return;
    }
    
    int64_t current_time = k_uptime_get();
    bool table_changed = false;
    bool should_publish = false;  // Flag to defer publication
    
    LOG_DBG("[Cleanup] Running cleanup check...");
    
    /* Lock forwarding table for thread-safe access */
    k_mutex_lock(&g_gradient_srv->forwarding_table_mutex, K_FOREVER);
    
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        const neighbor_entry_t *entry = nt_get(
            (const neighbor_entry_t *)g_gradient_srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
            i);
        
        if (entry == NULL) {
            continue;
        }
        
        if (nt_is_expired(
                (const neighbor_entry_t *)g_gradient_srv->forwarding_table,
                CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                i, current_time, CONFIG_BT_MESH_GRADIENT_SRV_NODE_TIMEOUT_MS)) {
            
            LOG_WRN("[Cleanup] Node 0x%04x expired (last seen %lld ms ago)",
                    entry->addr, current_time - entry->last_seen);
            
            /* Free backprop_dest linked list before removing entry */
            rrt_clear_entry(
                (void *)g_gradient_srv->forwarding_table,
                CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                i);
            
            uint16_t removed_addr = nt_remove(
                (neighbor_entry_t *)g_gradient_srv->forwarding_table,
                CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                i);
            
            if (removed_addr != GR_ADDR_UNASSIGNED) {
                LOG_INF("[Forwarding] Removed 0x%04x from index %d", removed_addr, i);
                table_changed = true;
                i--;  /* Re-check this index since entries shifted */
            }
        }
    }

    /* ========================================
     * Cleanup reverse routing table
     * ======================================== */
    int64_t rrt_timeout_ms = CONFIG_BT_MESH_GRADIENT_SRV_RRT_TIMEOUT_SEC * 1000LL;
    int rrt_removed = rrt_cleanup_expired(
        g_gradient_srv->forwarding_table,
        CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
        current_time,
        rrt_timeout_ms);
    
    if (rrt_removed > 0) {
        LOG_INF("[Cleanup] RRT: Removed %d expired reverse routes", rrt_removed);
    }

    /* Update gradient based on best remaining parent */
    if (table_changed) {
#ifdef CONFIG_BT_MESH_GRADIENT_SINK_NODE
        /* Sink node (Gateway) always has gradient=0, never update */
        LOG_DBG("[Cleanup] Sink node, gradient fixed at 0");
#else
        const neighbor_entry_t *best = nt_best(
            (const neighbor_entry_t *)g_gradient_srv->forwarding_table,
            CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE);

        if (best != NULL) {
            uint8_t best_parent_gradient = best->gradient;

            if (best_parent_gradient != UINT8_MAX) {
                uint8_t old_gradient = g_gradient_srv->gradient;
                uint8_t new_gradient = rp_compute_new_gradient(best_parent_gradient);
                
                /* Safety: Regular nodes cannot have gradient=0 */
                if (new_gradient == 0) {
                    new_gradient = 1;
                }
                
                g_gradient_srv->gradient = new_gradient;

                LOG_INF("[Cleanup] Gradient updated: [%d] -> [%d]",
                    old_gradient, g_gradient_srv->gradient);

                /* Notify heartbeat module of gradient change */
                heartbeat_update_gradient(g_gradient_srv->gradient);

                /* DEFER PUBLICATION */
                should_publish = true;
            }
        } else {
            LOG_WRN("[Cleanup] No parents available, resetting gradient to 255");
            g_gradient_srv->gradient = UINT8_MAX;
            /* Notify heartbeat module of gradient change */
            heartbeat_update_gradient(g_gradient_srv->gradient);
            /* DEFER PUBLICATION */
            should_publish = true;
        }
#endif
        
        LOG_INF("[Cleanup] Forwarding table after cleanup:");
        for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
            const neighbor_entry_t *entry = nt_get(
                (const neighbor_entry_t *)g_gradient_srv->forwarding_table,
                CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE,
                i);
            
            if (entry != NULL) {
                LOG_INF("  [%d] addr=0x%04x, gradient=%d, rssi=%d",
                        i, entry->addr, entry->gradient, entry->rssi);
            }
        }
    }

    /* Unlock forwarding table */
    k_mutex_unlock(&g_gradient_srv->forwarding_table_mutex);

    /* EXECUTE PUBLICATION SAFELY */
    if (should_publish) {
        bt_mesh_gradient_srv_gradient_send(g_gradient_srv);
    }

    k_work_schedule(&cleanup_work, K_MSEC(CLEANUP_INTERVAL_MS));
}
```

---

##### 6.3.5. File `src/model_handler.c`

**Mục đích:** Implement button handlers cho testing.

```c
#include "reverse_routing.h"
#include "heartbeat.h"

/**
 * @brief In debug info ra log
 *
 * Được gọi khi bấm Button 2
 */
static void print_debug_info(void)
{
    uint16_t my_addr = bt_mesh_model_elem(gradient_srv.model)->rt->addr;

    LOG_INF("╔════════════════════════════════════════════════════════════╗");
    LOG_INF("║                     DEBUG INFO                             ║");
    LOG_INF("╠════════════════════════════════════════════════════════════╣");

    /*
     * Node Info
     */
    LOG_INF("║ Node Address: 0x%04X", my_addr);
    LOG_INF("║ Gradient: %d %s",
            gradient_srv.gradient,
            gradient_srv.gradient == 0 ? "(GATEWAY)" : "");

    /*
     * Heartbeat Status
     */
    LOG_INF("║ Heartbeat: %s",
            heartbeat_is_active() ? "ACTIVE" : "INACTIVE");
    if (heartbeat_is_active()) {
        LOG_INF("║   Next heartbeat in: %lld ms",
                heartbeat_time_to_next());
    }

    LOG_INF("╠════════════════════════════════════════════════════════════╣");

    /*
     * Forwarding Table
     */
    LOG_INF("║ FORWARDING TABLE:");
    int fwd_count = 0;
    for (int i = 0; i < CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE; i++) {
        neighbor_entry_t *entry = &gradient_srv.fwd_table[i];
        if (entry->addr != BT_MESH_ADDR_UNASSIGNED) {
            int64_t age = k_uptime_get() - entry->last_seen;
            LOG_INF("║   [%d] 0x%04X  gradient=%d  age=%lld ms",
                    i, entry->addr, entry->gradient, age);
            fwd_count++;
        }
    }
    if (fwd_count == 0) {
        LOG_INF("║   (empty)");
    }

    LOG_INF("╠════════════════════════════════════════════════════════════╣");

    /*
     * Reverse Routing Table
     */
    LOG_INF("║ REVERSE ROUTING TABLE:");
    rrt_print_table(gradient_srv.fwd_table);

    LOG_INF("╚════════════════════════════════════════════════════════════╝");
}

/**
 * @brief Button event handler
 *
 * @param button_state  Current state of all buttons
 * @param has_changed   Bitmap of buttons that changed state
 */
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    uint16_t my_addr = bt_mesh_model_elem(gradient_srv.model)->rt->addr;

    /*
     * Button 1: Action
     */
    if (has_changed & DK_BTN1_MSK) {
        if (button_state & DK_BTN1_MSK) {
            /* Button 1 pressed */

            if (gradient_srv.gradient == 0) {
                /*
                 * ╔═══════════════════════════════════════════════════════╗
                 * ║ GATEWAY: Gửi BACKPROP đến node bất kỳ                 ║
                 * ╚═══════════════════════════════════════════════════════╝
                 */
                uint16_t dest;
                int err = rrt_get_any_destination(gradient_srv.fwd_table, &dest);

                if (err == 0) {
                    LOG_INF("Button 1: Sending BACKPROP to 0x%04X", dest);

                    /* Gửi test payload = 0xBEEF */
                    err = bt_mesh_gradient_srv_backprop_send(&gradient_srv, dest, 0xBEEF);

                    if (err) {
                        LOG_ERR("Button 1: Failed to send BACKPROP (err=%d)", err);
                    } else {
                        LOG_INF("Button 1: BACKPROP sent successfully");
                    }
                } else {
                    LOG_WRN("Button 1: No destinations in RRT");
                    LOG_WRN("         Wait for nodes to send heartbeat");
                }

            } else {
                /*
                 * ╔═══════════════════════════════════════════════════════╗
                 * ║ REGULAR NODE: Gửi DATA lên Gateway                    ║
                 * ╚═══════════════════════════════════════════════════════╝
                 */
                LOG_INF("Button 1: Sending DATA (0x%04X)", my_addr);

                /* Gửi địa chỉ của chính mình làm test data */
                int err = data_forward_send_direct(&gradient_srv, my_addr);

                if (err) {
                    LOG_ERR("Button 1: Failed to send DATA (err=%d)", err);
                } else {
                    LOG_INF("Button 1: DATA sent successfully");
                }
            }
        }
    }

    /*
     * Button 2: Debug Info
     */
    if (has_changed & DK_BTN2_MSK) {
        if (button_state & DK_BTN2_MSK) {
            /* Button 2 pressed */
            LOG_INF("Button 2: Printing debug info...");
            print_debug_info();
        }
    }
}
```

---

##### 6.3.6. File `prj.conf`

**Mục đích:** Thêm config cần thiết.

```
# Heap memory pool cho k_malloc/k_free
# Cần cho dynamic allocation trong RRT
CONFIG_HEAP_MEM_POOL_SIZE=4096

# Logging (optional, để debug dễ hơn)
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
```

---

##### 6.3.7. File `boards/nrf54l15dk_nrf54l15_cpuapp_ns.conf` (FILE MỚI)

**Mục đích:** Board config cho non-secure variant.

```
# Board-specific config for nRF54L15 DK (non-secure)

# TrustZone non-secure configuration
CONFIG_TRUSTED_EXECUTION_NONSECURE=y

# Flash configuration for non-secure
CONFIG_FLASH_BASE_ADDRESS=0x0

# Additional settings specific to NS variant
```

---

#### 6.4. Ví Dụ Hoạt Động Chi Tiết

##### 6.4.1. Cleanup Scenario

```
═══════════════════════════════════════════════════════════════════════
SCENARIO: RRT Cleanup khi node offline
═══════════════════════════════════════════════════════════════════════

Initial State:
- Gateway có route đến NodeC qua NodeA
- Route last_seen = T=0

Timeline:
┌────────┬────────────────────────────────────────────────────────────┐
│ Time   │ Event                                                      │
├────────┼────────────────────────────────────────────────────────────┤
│ T=0s   │ NodeC gửi heartbeat                                        │
│        │ → Route to NodeC: last_seen = 0s                           │
├────────┼────────────────────────────────────────────────────────────┤
│ T=15s  │ Cleanup runs (interval = 15s)                              │
│        │ → Route age = 15s < 90s (timeout)                          │
│        │ → Route giữ nguyên                                         │
├────────┼────────────────────────────────────────────────────────────┤
│ T=30s  │ NodeC gửi heartbeat                                        │
│        │ → Route to NodeC: last_seen = 30s (refresh)                │
├────────┼────────────────────────────────────────────────────────────┤
│ T=35s  │ NodeC TẮT NGUỒN                                            │
├────────┼────────────────────────────────────────────────────────────┤
│ T=45s  │ Cleanup runs                                               │
│        │ → Route age = 45s - 30s = 15s < 90s                        │
│        │ → Route giữ nguyên                                         │
├────────┼────────────────────────────────────────────────────────────┤
│ T=60s  │ Heartbeat expected nhưng KHÔNG ĐẾN                         │
│        │ Cleanup runs                                               │
│        │ → Route age = 60s - 30s = 30s < 90s                        │
│        │ → Route giữ nguyên (cho phép miss 1 heartbeat)             │
├────────┼────────────────────────────────────────────────────────────┤
│ T=90s  │ Heartbeat expected nhưng KHÔNG ĐẾN                         │
│        │ Cleanup runs                                               │
│        │ → Route age = 90s - 30s = 60s < 90s                        │
│        │ → Route giữ nguyên (cho phép miss 2 heartbeat)             │
├────────┼────────────────────────────────────────────────────────────┤
│ T=120s │ Heartbeat expected nhưng KHÔNG ĐẾN                         │
│        │ Cleanup runs                                               │
│        │ → Route age = 120s - 30s = 90s = 90s (timeout)             │
│        │ → ROUTE BỊ XÓA!                                            │
│        │ → Log: "RRT cleanup: Removing dest 0x0004 (age: 90000 ms)" │
├────────┼────────────────────────────────────────────────────────────┤
│ T=125s │ Gateway muốn gửi BACKPROP đến NodeC                        │
│        │ → rrt_find_nexthop() returns BT_MESH_ADDR_UNASSIGNED       │
│        │ → "No route to destination 0x0004"                         │
│        │ → Gateway biết NodeC không available                       │
└────────┴────────────────────────────────────────────────────────────┘
```

##### 6.4.2. Button Testing Scenario

```
═══════════════════════════════════════════════════════════════════════
SCENARIO: Testing với buttons trên Gateway và Node
═══════════════════════════════════════════════════════════════════════

Setup:
- Gateway: nRF54L15 DK #1, kết nối UART COM3
- NodeC: nRF54L15 DK #2, kết nối UART COM4

Timeline:
┌────────┬────────────────────────────────────────────────────────────┐
│ Time   │ Event                                                      │
├────────┼────────────────────────────────────────────────────────────┤
│ T=0s   │ Cả hai boards khởi động, được provisioned                  │
├────────┼────────────────────────────────────────────────────────────┤
│ T=5s   │ [NodeC] Bấm Button 1                                       │
│        │ → Gửi DATA [src=0x0004, data=0x0004]                       │
│        │ → Gateway nhận, học route đến NodeC                        │
├────────┼────────────────────────────────────────────────────────────┤
│ T=10s  │ [Gateway] Bấm Button 2                                     │
│        │ → In debug info:                                           │
│        │   ╔══════════════════════════════════════╗                 │
│        │   ║         DEBUG INFO                   ║                 │
│        │   ╠══════════════════════════════════════╣                 │
│        │   ║ Node Address: 0x0001                 ║                 │
│        │   ║ Gradient: 0 (GATEWAY)                ║                 │
│        │   ║ Heartbeat: INACTIVE                  ║                 │
│        │   ╠══════════════════════════════════════╣                 │
│        │   ║ FORWARDING TABLE:                    ║                 │
│        │   ║   [0] 0x0002  gradient=1  age=5000ms ║                 │
│        │   ╠══════════════════════════════════════╣                 │
│        │   ║ REVERSE ROUTING TABLE:               ║                 │
│        │   ║ Neighbor 0x0002:                     ║                 │
│        │   ║   -> 0x0004 (age: 5000 ms)           ║                 │
│        │   ╚══════════════════════════════════════╝                 │
├────────┼────────────────────────────────────────────────────────────┤
│ T=15s  │ [Gateway] Bấm Button 1                                     │
│        │ → rrt_get_any_destination() returns 0x0004                 │
│        │ → Gửi BACKPROP [dest=0x0004, ttl=10, data=0xBEEF]          │
│        │ → Log: "Button 1: Sending BACKPROP to 0x0004"              │
├────────┼────────────────────────────────────────────────────────────┤
│ T=15s  │ [NodeC] Nhận BACKPROP                                      │
│        │ → dest=0x0004 == my_addr                                   │
│        │ → Callback: data_received(srv, 0x0002, 0xBEEF)             │
│        │ → Log:                                                     │
│        │   ╔══════════════════════════════════════╗                 │
│        │   ║     BACKPROP DATA RECEIVED!          ║                 │
│        │   ║ From (last hop): 0x0002              ║                 │
│        │   ║ Data: 0xBEEF                         ║                 │
│        │   ╚══════════════════════════════════════╝                 │
├────────┼────────────────────────────────────────────────────────────┤
│ T=20s  │ [NodeC] Bấm Button 2                                       │
│        │ → In debug info cho NodeC                                  │
│        │ → Confirm gradient=3, heartbeat ACTIVE                     │
└────────┴────────────────────────────────────────────────────────────┘
```

---

#### 6.5. Flow Diagram - Cleanup

```
                    ┌───────────────────────────────────────┐
                    │      cleanup_handler() chạy           │
                    │      (mỗi 15 giây)                    │
                    └───────────────────┬───────────────────┘
                                        │
              ┌─────────────────────────┴─────────────────────────┐
              │                                                   │
              ▼                                                   ▼
    ┌─────────────────────┐                         ┌─────────────────────┐
    │ Cleanup Forwarding  │                         │ Cleanup RRT         │
    │ Table               │                         │                     │
    └──────────┬──────────┘                         └──────────┬──────────┘
               │                                               │
               ▼                                               ▼
    ┌─────────────────────────┐                   ┌─────────────────────────┐
    │ Duyệt từng neighbor     │                   │ rrt_cleanup_expired()   │
    │ entry                   │                   │ với timeout = 90s       │
    └──────────┬──────────────┘                   └──────────┬──────────────┘
               │                                              │
               ▼                                              ▼
    ┌─────────────────────────┐                   ┌─────────────────────────┐
    │ age > fwd_timeout?      │                   │ Duyệt từng destination  │
    │ (e.g., 120s)            │                   │ trong RRT               │
    └──────────┬──────────────┘                   └──────────┬──────────────┘
               │                                              │
        ┌──────┴──────┐                                ┌──────┴──────┐
        │             │                                │             │
       YES           NO                               YES           NO
        │             │                                │             │
        ▼             ▼                                ▼             ▼
┌──────────────┐ ┌──────────┐                 ┌──────────────┐ ┌──────────┐
│Clear RRT     │ │Keep      │                 │Remove entry  │ │Keep      │
│entries cho   │ │entry     │                 │k_free()      │ │entry     │
│neighbor này  │ │          │                 │              │ │          │
│              │ │          │                 │              │ │          │
│Remove        │ │          │                 │              │ │          │
│neighbor      │ │          │                 │              │ │          │
└──────────────┘ └──────────┘                 └──────────────┘ └──────────┘
               │                                              │
               └──────────────────┬───────────────────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────────────────┐
                    │      Reschedule cleanup               │
                    │      sau 15 giây                      │
                    └───────────────────────────────────────┘
```

---

#### 6.6. Tóm Tắt Thay Đổi

| File | Loại | Thay Đổi |
|------|------|----------|
| `Kconfig` | Sửa | Thêm `RRT_TIMEOUT_SEC` option (default 90) |
| `include/reverse_routing.h` | Sửa | Thêm `rrt_get_any_destination()` prototype |
| `src/reverse_routing.c` | Sửa | Implement `rrt_get_any_destination()` |
| `src/gradient_work.c` | Sửa | Gọi `rrt_cleanup_expired()` trong cleanup |
| `src/model_handler.c` | Sửa | Button 1: BACKPROP (Gateway) / DATA (Node) |
| `src/model_handler.c` | Sửa | Button 2: Debug info |
| `src/model_handler.c` | Sửa | Thêm `print_debug_info()` helper |
| `prj.conf` | Sửa | Thêm `CONFIG_HEAP_MEM_POOL_SIZE=4096` |
| `boards/nrf54l15dk_nrf54l15_cpuapp_ns.conf` | **MỚI** | Board config cho NS variant |

---

#### 6.7. Kết Quả

Sau khi implement giai đoạn 6:

✅ RRT cleanup tự động mỗi 15 giây  
✅ Routes expired (>90s) bị xóa  
✅ Button 1: Gateway gửi BACKPROP, Node gửi DATA  
✅ Button 2: In debug info đầy đủ  
✅ Helper function `rrt_get_any_destination()` cho testing  
✅ Memory pool enabled cho k_malloc  
✅ Board config cho nRF54L15 NS variant  
✅ **HỆ THỐNG HOÀN CHỈNH VÀ SẴN SÀNG TEST!**

---

#### 6.8. Lưu Ý Quan Trọng

1. **Cleanup Order:**
   ```
   Quan trọng: Clear RRT entries TRƯỚC KHI xóa neighbor
   
   Sai:
   1. Xóa neighbor entry
   2. RRT entries vẫn trỏ đến memory đã free → Crash!
   
   Đúng:
   1. rrt_clear_entry(neighbor) - giải phóng linked list
   2. Reset neighbor entry
   ```

2. **Timeout Relationship:**
   ```
   RRT_TIMEOUT > FWD_TABLE_TIMEOUT có thể gây vấn đề:
   - Neighbor bị xóa nhưng RRT entries vẫn còn
   - Lookup sẽ fail vì neighbor không còn
   
   Khuyến nghị:
   RRT_TIMEOUT ≤ FWD_TABLE_TIMEOUT
   hoặc clear RRT khi xóa neighbor (đã implement)
   ```

3. **Heap Size:**
   ```
   4096 bytes heap cho:
   - backprop_node_t = ~12 bytes (2 + 8 + pointer)
   - Tối đa ~300 RRT entries
   - Đủ cho mạng 50+ nodes
   
   Nếu cần nhiều hơn, tăng CONFIG_HEAP_MEM_POOL_SIZE
   ```

4. **Button Debouncing:**
   ```
   DK library đã xử lý debouncing
   Không cần thêm debouncing trong button_handler
   Chỉ xử lý khi has_changed bit được set
   ```

---

## Tổng Kết Các File Đã Thay Đổi/Tạo Mới

### Files mới tạo:
| File | Mô tả |
|------|-------|
| `include/reverse_routing.h` | API cho Reverse Routing Table |
| `src/reverse_routing.c` | Implementation RRT |
| `include/heartbeat.h` | API cho Heartbeat module |
| `src/heartbeat.c` | Implementation Heartbeat |
| `boards/nrf54l15dk_nrf54l15_cpuapp_ns.conf` | Board config cho NS variant |

### Files đã sửa:
| File | Thay Đổi |
|------|----------|
| `include/gradient_types.h` | Thêm `backprop_dest` pointer |
| `include/gradient_srv.h` | Thêm opcode, constant, prototype, callback |
| `include/data_forward.h` | Cập nhật prototype với `original_source` |
| `src/gradient_srv.c` | Thêm handler, route learning, backprop send |
| `src/data_forward.c` | Sửa packet format (4 bytes) |
| `src/model_handler.c` | Thêm init, button handler, callback |
| `src/gradient_work.c` | Thêm RRT cleanup, heartbeat update |
| `src/neighbor_table.c` | Thêm `backprop_dest` init/cleanup |
| `CMakeLists.txt` | Thêm source files |
| `Kconfig` | Thêm config options |
| `prj.conf` | Thêm heap config |

---

## Flow Dữ Liệu

### Uplink (Node → Gateway):
```
Node C [gradient=3]
  │
  │ DATA [original_src=0x0004, data=0x1234]
  ▼
Node B [gradient=2]
  │── Học: "Đến 0x0004, gửi qua 0x0004"
  │
  │ DATA [original_src=0x0004, data=0x1234]
  ▼
Node A [gradient=1]
  │── Học: "Đến 0x0004, gửi qua 0x0003"
  │
  │ DATA [original_src=0x0004, data=0x1234]
  ▼
Gateway [gradient=0]
  │── Học: "Đến 0x0004, gửi qua 0x0002"
  │── Nhận data 0x1234 từ Node C
```

### Downlink (Gateway → Node):
```
Gateway [gradient=0]
  │── Tìm route đến 0x0004: Nexthop = 0x0002
  │
  │ BACKPROP [dest=0x0004, ttl=10, payload=0xABCD]
  ▼
Node A [gradient=1]
  │── Tìm route đến 0x0004: Nexthop = 0x0003
  │
  │ BACKPROP [dest=0x0004, ttl=9, payload=0xABCD]
  ▼
Node B [gradient=2]
  │── Tìm route đến 0x0004: Nexthop = 0x0004
  │
  │ BACKPROP [dest=0x0004, ttl=8, payload=0xABCD]
  ▼
Node C [gradient=3]
  │── dest == my_addr
  │── Deliver: Nhận payload 0xABCD
```

### Heartbeat:
```
Mỗi 30 giây:
  Node C gửi DATA [original_src=0x0004, data=0xFFFF]
    → Các node và Gateway refresh route đến Node C
    → Route không bị expired (timeout 90 giây)
```

---

## Testing Checklist

### Chuẩn bị:
- [ ] Flash Gateway và 2+ nodes
- [ ] Kết nối UART từ laptop đến các boards
- [ ] Mở terminal (115200 baud) cho mỗi board

### Test Uplink (DATA):
- [ ] Trên Node: Bấm Button 1
- [ ] Kiểm tra log trên Gateway: "DATA received from 0x00XX"
- [ ] Trên Gateway: Bấm Button 2, xác nhận RRT có route đến node

### Test Heartbeat:
- [ ] Đợi 30 giây
- [ ] Trên Gateway: Bấm Button 2
- [ ] Xác nhận RRT có routes đến tất cả nodes
- [ ] Kiểm tra log "Heartbeat received from 0x00XX"

### Test Downlink (BACKPROP):
- [ ] Trên Gateway: Bấm Button 1
- [ ] Kiểm tra log "Sending BACKPROP to 0x00XX"
- [ ] Trên Node đích: Kiểm tra log "BACKPROP DATA RECEIVED!"
- [ ] Xác nhận payload = 0xBEEF

### Test Route Expiration:
- [ ] Tắt một node
- [ ] Đợi 90 giây (3x heartbeat interval)
- [ ] Trên Gateway: Bấm Button 2
- [ ] Xác nhận route đến node đã bị xóa
- [ ] Bật lại node
- [ ] Đợi 30 giây
- [ ] Xác nhận route được học lại

---

## Cấu Hình

### Kconfig options:
| Option | Default | Mô tả |
|--------|---------|-------|
| `CONFIG_BT_MESH_GRADIENT_SRV_FORWARDING_TABLE_SIZE` | 3 | Số entry trong forwarding table |
| `CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_ENABLED` | y | Bật/tắt heartbeat |
| `CONFIG_BT_MESH_GRADIENT_SRV_HEARTBEAT_INTERVAL_SEC` | 30 | Khoảng cách heartbeat (giây) |
| `CONFIG_BT_MESH_GRADIENT_SRV_RRT_TIMEOUT_SEC` | 90 | Timeout cho RRT entry (giây) |
| `CONFIG_BT_MESH_GRADIENT_SINK_NODE` | n | Đánh dấu node là Gateway |
| `CONFIG_HEAP_MEM_POOL_SIZE` | 4096 | Kích thước heap cho k_malloc |

### Giá trị mặc định hợp lý:
| Parameter | Value | Lý do |
|-----------|-------|-------|
| Heartbeat interval | 30 giây | Cân bằng giữa freshness và traffic |
| RRT timeout | 90 giây | 3× heartbeat, cho phép miss 2 lần |
| TTL | 10 | Đủ cho mạng 10 hop |
| Heap | 4096 bytes | Đủ cho ~100 route entries |

---

## Lưu Ý Quan Trọng

1. **Gateway duy nhất:**
   - Chỉ một node được config là Gateway (`CONFIG_BT_MESH_GRADIENT_SINK_NODE=y`)
   - Gateway có gradient = 0

2. **Gateway không gửi heartbeat:**
   - Gateway là đích của tất cả DATA packets
   - Không cần duy trì route "đến Gateway"

3. **RRT chỉ học từ DATA packets:**
   - Không học từ BACKPROP
   - Không học từ GRADIENT messages
   - Heartbeat được tính là DATA

4. **TTL chống loop:**
   - TTL giảm mỗi hop
   - Packet bị drop khi TTL ≤ 1
   - Tránh loop vô hạn trong mạng

5. **Cleanup timing:**
   - Cleanup chạy mỗi 15 giây
   - Kiểm tra cả forwarding table và RRT
   - Clear RRT trước khi xóa neighbor

6. **Random delay cho heartbeat:**
   - Tránh collision khi nhiều nodes khởi động cùng lúc
   - Delay 0-10 giây khi start
   - Sau đó cố định mỗi 30 giây
