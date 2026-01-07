# 📚 Tổng hợp Chi tiết Các Module - Gradient Routing

---

## 1️⃣ **gradient_types.h** - Định nghĩa Kiểu dữ liệu chung

| Thành phần | Mô tả |
|------------|-------|
| `neighbor_entry_t` | Struct chứa thông tin 1 neighbor trong forwarding table |
| `GR_ADDR_UNASSIGNED` | Hằng số địa chỉ không hợp lệ (0x0000) |

```c
typedef struct {
    uint16_t addr;      // Địa chỉ mesh của neighbor
    uint8_t  gradient;  // Giá trị gradient
    int8_t   rssi;      // Cường độ tín hiệu (dBm)
    int64_t  last_seen; // Timestamp (ms)
} neighbor_entry_t;
```

---

## 2️⃣ **neighbor_table.h/c** - Quản lý Forwarding Table

| Function | Tham số | Return | Mô tả |
|----------|---------|--------|-------|
| `nt_init()` | `table, size` | `void` | Clear toàn bộ bảng |
| `nt_update_sorted()` | `table, size, addr, gradient, rssi, timestamp` | `void` | Thêm/cập nhật neighbor, tự sắp xếp |
| `nt_best()` | `table, size` | `neighbor_entry_t*` | Lấy best parent (gradient thấp nhất) |
| `nt_get()` | `table, size, index` | `neighbor_entry_t*` | Lấy entry tại index |
| `nt_remove()` | `table, size, index` | `uint16_t` | Xóa entry, trả về addr đã xóa |
| `nt_count()` | `table, size` | `int` | Đếm số entry hợp lệ |
| `nt_is_expired()` | `table, size, index, current_time, timeout_ms` | `bool` | Kiểm tra entry đã timeout |

**Đặc điểm:**
- Bảng luôn được sắp xếp theo gradient tăng dần
- `table[0]` luôn là best parent
- Tự động shift entries khi xóa

---

## 3️⃣ **routing_policy.h/c** - Logic Quyết định Định tuyến

| Function | Tham số | Return | Mô tả |
|----------|---------|--------|-------|
| `rp_is_candidate_acceptable()` | `rssi` | `bool` | RSSI ≥ -60 dBm? |
| `rp_should_process_gradient()` | `received_gradient, my_gradient` | `bool` | received < my? |
| `rp_compute_new_gradient()` | `best_parent_gradient` | `uint8_t` | parent + 1 |
| `rp_should_update_my_gradient()` | `my_gradient, best_parent_gradient` | `bool` | (best+1) < my? |

**Hằng số:**
```c
#define RP_RSSI_THRESHOLD    (-60)  // dBm
#define RP_GRADIENT_SINK     0
#define RP_GRADIENT_INFINITE UINT8_MAX
```

---

## 4️⃣ **led_indication.h/c** - LED Feedback

| Function | Tham số | Return | Mô tả |
|----------|---------|--------|-------|
| `led_indication_init()` | `void` | `void` | Khởi tạo work items cho LED |
| `led_indicate_gradient_received()` | `void` | `void` | Nhận gradient → LED3 blink 3x |
| `led_indicate_data_forwarded()` | `void` | `void` | Forward data → LED2 blink 3x |
| `led_indicate_sink_received()` | `void` | `void` | Sink nhận data → LED1 toggle |
| `led_indicate_attention()` | `bool on` | `void` | Attention mode → LED pattern chạy vòng |

**LED Assignment:**
| LED | Chức năng |
|-----|-----------|
| `DK_LED1` | Sink received |
| `DK_LED2` | Data forwarded |
| `DK_LED3` | Gradient received |
| `DK_LED4` | Attention pattern |

**Pattern:**
- Blink: 3 lần, 100ms interval
- Attention: 4-LED pattern, 30ms interval

---

## 5️⃣ **data_forward.h/c** - Forward Data với Retry

| Function | Tham số | Return | Mô tả |
|----------|---------|--------|-------|
| `data_forward_init()` | `void` | `void` | Khởi tạo retry work |
| `data_forward_send()` | `gradient_srv, data, sender_addr` | `int` | Forward với retry, skip sender |
| `data_forward_send_direct()` | `gradient_srv, addr, data` | `int` | Gửi trực tiếp không retry |

**Internal functions:**
| Function | Mô tả |
|----------|-------|
| `data_send_internal()` | Gửi BT Mesh message với callback |
| `data_send_end_cb()` | TX complete callback, trigger retry nếu fail |
| `data_retry_handler()` | Work handler cho retry logic |

**Retry Logic:**
```
1. Gửi đến table[0] (best parent)
2. Nếu TX fail → data_retry_handler()
3. current_index++ → thử table[1]
4. Skip nếu addr == sender_addr
5. Tiếp tục cho đến hết bảng hoặc thành công
```

---

## 6️⃣ **gradient_work.h/c** - Workqueue Management

| Function | Tham số | Return | Mô tả |
|----------|---------|--------|-------|
| `gradient_work_init()` | `void` | `void` | Khởi tạo các work items |
| `gradient_work_set_srv()` | `gradient_srv` | `void` | Set global reference |
| `gradient_work_start_cleanup()` | `void` | `void` | Start cleanup timer (15s cycle) |
| `gradient_work_schedule_initial_publish()` | `void` | `void` | Schedule publish sau 500ms |
| `gradient_work_schedule_process()` | `gradient_srv, gradient, sender_addr, rssi` | `void` | Schedule xử lý gradient |

**Internal handlers:**
| Handler | Trigger | Mô tả |
|---------|---------|-------|
| `initial_publish_handler()` | Sau 500ms từ start | Publish gradient lần đầu |
| `gradient_process_handler()` | Khi nhận gradient message | Update table & gradient |
| `cleanup_handler()` | Mỗi 15s | Xóa expired nodes (timeout 30s) |

**Cleanup Logic:**
```
1. Duyệt forwarding_table
2. Kiểm tra nt_is_expired() với timeout 30s
3. Xóa expired entries bằng nt_remove()
4. Nếu table thay đổi → recalculate gradient
5. Re-schedule cleanup sau 15s
```

---

## 7️⃣ **gradient_srv.h/c** - Bluetooth Mesh Vendor Model

### Public API:
| Function | Tham số | Return | Mô tả |
|----------|---------|--------|-------|
| `bt_mesh_gradient_srv_gradient_send()` | `gradient_srv` | `int` | Publish gradient message |
| `bt_mesh_gradient_srv_data_send()` | `gradient_srv, addr, data` | `int` | Gửi data message |

### Internal Message Handlers:
| Handler | Opcode | Mô tả |
|---------|--------|-------|
| `handle_gradient_mesage()` | `0x0A` | Nhận gradient beacon |
| `handle_data_message()` | `0x0B` | Nhận/forward data |

### Model Callbacks:
| Callback | Trigger | Mô tả |
|----------|---------|-------|
| `bt_mesh_gradient_srv_init()` | Model init | Init pub buffer, init sub-modules |
| `bt_mesh_gradient_srv_start()` | Model start | Set global srv, configure pub, start timers |
| `bt_mesh_gradient_srv_update_handler()` | Auto-publish | Encode gradient vào pub buffer |
| `bt_mesh_gradient_srv_reset()` | Node reset | Clear settings |

**Opcodes:**
```c
#define BT_MESH_GRADIENT_SRV_OP_GRADIENT_STATUS  0x0A
#define BT_MESH_GRADIENT_SRV_OP_DATA_MESSAGE     0x0B
```

---

## 8️⃣ **model_handler.h/c** - Application Layer

| Function | Tham số | Return | Mô tả |
|----------|---------|--------|-------|
| `model_handler_init()` | `void` | `bt_mesh_comp*` | Khởi tạo models, buttons, table |

### Components:
| Component | Mô tả |
|-----------|-------|
| `gradient_srv` | Instance của vendor model |
| `health_srv` | Health server với attention callbacks |
| `elements[]` | Mesh element composition |
| `button_handler()` | Button 1 → gửi data |
| `attention_on/off()` | Delegate cho `led_indicate_attention()` |

**Initialization:**
```c
1. Shell backend init
2. Forwarding table clear (all entries = UNASSIGNED)
3. Set gradient (0 nếu SINK, 255 nếu regular)
4. Button init
5. Return mesh composition
```

---

## 📊 Sơ đồ Dependencies

```
┌─────────────────────────────────────────────────────────────┐
│                        main.c                               │
└─────────────────────────────┬───────────────────────────────┘
                              │ calls model_handler_init()
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    model_handler.c                          │
│  - gradient_srv instance                                    │
│  - button_handler()                                         │
│  - attention callbacks → led_indication                     │
└─────────────────────────────┬───────────────────────────────┘
                              │ includes
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                     gradient_srv.c                          │
│  - Message handlers                                         │
│  - Model callbacks                                          │
│  - Public APIs                                              │
├──────────┬──────────┬──────────┬──────────┬────────────────┤
│          │          │          │          │                │
│    ┌─────▼────┐ ┌───▼────┐ ┌───▼────┐ ┌───▼────┐          │
│    │neighbor_ │ │routing_│ │led_    │ │data_   │          │
│    │table.c   │ │policy.c│ │indica- │ │forward │          │
│    │          │ │        │ │tion.c  │ │.c      │          │
│    └──────────┘ └────────┘ └────────┘ └────────┘          │
│                                              │              │
│                              ┌───────────────▼────────────┐│
│                              │      gradient_work.c       ││
│                              │  - cleanup_handler         ││
│                              │  - gradient_process_handler││
│                              │  - initial_publish_handler ││
│                              └────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    gradient_types.h                         │
│  - neighbor_entry_t                                         │
│  - GR_ADDR_UNASSIGNED                                       │
└─────────────────────────────────────────────────────────────┘
```

---

## 📈 Tổng số dòng code (ước tính)

| Module | Header | Source | Tổng |
|--------|--------|--------|------|
| gradient_types | 30 | - | 30 |
| neighbor_table | 50 | 130 | 180 |
| routing_policy | 40 | 50 | 90 |
| led_indication | 56 | 110 | 166 |
| data_forward | 45 | 200 | 245 |
| gradient_work | 45 | 210 | 255 |
| gradient_srv | 90 | 220 | 310 |
| model_handler | 20 | 190 | 210 |
| **Tổng** | **376** | **1110** | **~1486** |
