/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "neighbor_table.h"
#include <limits.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(neighbor_table, LOG_LEVEL_DBG);

void nt_init(neighbor_entry_t *table, size_t table_size)
{
    if (table == NULL || table_size == 0) {
        return;
    }

    for (size_t i = 0; i < table_size; i++) {
        table[i].addr = GR_ADDR_UNASSIGNED;
        table[i].rssi = INT8_MIN;
        table[i].gradient = UINT8_MAX;
        table[i].last_seen = 0;
        table[i].backprop_dest = NULL; /* QUAN TRỌNG: Init NULL để tránh Kernel Panic */
    }
}

bool nt_update_sorted(neighbor_entry_t *table, size_t table_size,
                      uint16_t sender_addr, uint8_t sender_gradient, int8_t sender_rssi,
                      int64_t now_ms)
{
    if (table == NULL || table_size == 0) {
        return false;
    }

    int existing_pos = -1;
    struct backprop_node *saved_backprop = NULL; 

    /* 1. Tìm xem node đã tồn tại chưa */
    for (size_t i = 0; i < table_size; i++) {
        if (table[i].addr == sender_addr) {
            existing_pos = i;
            /* Lưu lại danh sách RRT cũ (Linked List) để không bị mất khi sort/move */
            saved_backprop = table[i].backprop_dest;
            break;
        }
    }

    /* 2. Xác định vị trí chèn mới dựa trên thuật toán Gradient */
    int insert_pos = table_size; // Mặc định là cuối bảng
    
    for (size_t i = 0; i < table_size; i++) {
        // Nếu gặp slot trống -> chèn vào đây luôn (vì bảng đã sort)
        if (table[i].addr == GR_ADDR_UNASSIGNED) {
            if (insert_pos > i) insert_pos = i;
            break;
        }

        // So sánh tiêu chí: Gradient nhỏ hơn -> Tốt hơn -> Đứng trước
        if (sender_gradient < table[i].gradient) {
            insert_pos = i;
            break;
        }
        // Gradient bằng nhau -> RSSI lớn hơn -> Tốt hơn -> Đứng trước
        else if (sender_gradient == table[i].gradient) {
            if (sender_rssi > table[i].rssi) {
                insert_pos = i;
                break;
            }
        }
        // Nếu Gradient lớn hơn -> Tiếp tục tìm
    }

    /* Nếu bảng đầy và vị trí chèn nằm ngoài bảng (và node này không có trong bảng) 
       -> Node này tệ hơn tất cả -> Bỏ qua */
    if (insert_pos >= table_size && existing_pos == -1) {
        return false;
    }

    /* 3. Thực hiện dịch chuyển mảng (Shift) */
    
    // TRƯỜNG HỢP A: Node mới hoàn toàn
    if (existing_pos == -1) {
        // Dịch các phần tử từ insert_pos về sau lùi 1 bước để tạo chỗ trống
        for (int i = table_size - 1; i > insert_pos; i--) {
            table[i] = table[i - 1]; 
        }
        
        // Tại vị trí insert_pos, reset backprop_dest vì đây là node mới
        // QUAN TRỌNG: Ngăn chặn dùng lại pointer rác của node cũ tại vị trí này
        table[insert_pos].backprop_dest = NULL; 
    } 
    // TRƯỜNG HỢP B: Node đã tồn tại, cần cập nhật vị trí
    else {
        // Nếu vị trí không đổi -> Chỉ update info
        if (existing_pos == insert_pos) {
            table[insert_pos].gradient = sender_gradient;
            table[insert_pos].rssi = sender_rssi;
            table[insert_pos].last_seen = now_ms;
            // backprop_dest giữ nguyên
            return true;
        }

        // Nếu vị trí thay đổi, cần dịch chuyển
        if (existing_pos > insert_pos) {
            // Node tốt lên -> Dịch các node ở giữa xuống dưới
            for (int i = existing_pos; i > insert_pos; i--) {
                table[i] = table[i - 1];
            }
        } else {
            // Node tệ đi -> Dịch các node ở giữa lên trên để lấp chỗ existing_pos
            for (int i = existing_pos; i < insert_pos; i++) {
                // Kiểm tra biên để tránh truy cập ngoài mảng (dù logic insert_pos thường an toàn)
                if (i + 1 < table_size) {
                    table[i] = table[i + 1];
                }
            }
            // FIX LOGIC SORT: Khi dịch chuyển lấp chỗ cũ, index insert_pos thực tế đã bị lùi 1 đơn vị
            if (insert_pos > 0) {
                insert_pos--; 
            }
        }
        
        // Gán lại pointer đã lưu (QUAN TRỌNG: Giữ lại routing ngược của node này)
        table[insert_pos].backprop_dest = saved_backprop;
    }

    /* 4. Cập nhật thông tin tại insert_pos */
    if (insert_pos < table_size) {
        table[insert_pos].addr = sender_addr;
        table[insert_pos].gradient = sender_gradient;
        table[insert_pos].rssi = sender_rssi;
        table[insert_pos].last_seen = now_ms;
        // backprop_dest đã được xử lý ở bước 3
    }

    return true;
}

const neighbor_entry_t *nt_get(const neighbor_entry_t *table, size_t table_size, size_t index)
{
    if (table == NULL || index >= table_size) {
        return NULL;
    }
    
    if (table[index].addr == GR_ADDR_UNASSIGNED) {
        return NULL;
    }

    return &table[index];
}

uint16_t nt_remove(neighbor_entry_t *table, size_t table_size, size_t idx)
{
    if (table == NULL || idx >= table_size) {
        return GR_ADDR_UNASSIGNED;
    }

    uint16_t removed_addr = table[idx].addr;

    /* Dịch chuyển các phần tử phía sau lên */
    for (size_t i = idx; i < table_size - 1; i++) {
        table[i] = table[i + 1];
    }

    /* Reset phần tử cuối cùng */
    size_t last = table_size - 1;
    table[last].addr = GR_ADDR_UNASSIGNED;
    table[last].rssi = INT8_MIN;
    table[last].gradient = UINT8_MAX;
    table[last].last_seen = 0;
    table[last].backprop_dest = NULL; /* QUAN TRỌNG: Phải set NULL */

    return removed_addr;
}

/* [FIX] Hàm này đã được sửa signature để khớp với header */
bool nt_is_expired(const neighbor_entry_t *table, size_t table_size, size_t idx, 
                   int64_t timeout, int64_t now)
{
    if (table == NULL || idx >= table_size) {
        return false;
    }
    
    const neighbor_entry_t *entry = &table[idx];
    
    if (entry->addr == GR_ADDR_UNASSIGNED) {
        return false;
    }
    
    return (now - entry->last_seen) > timeout;
}

/* [FIX] Hàm này đã được sửa signature để khớp với header (const) */
const neighbor_entry_t *nt_best(const neighbor_entry_t *table, size_t table_size)
{
    if (table == NULL || table_size == 0) return NULL;
    
    /* Vì bảng đã được sắp xếp (Sorted Table) bởi nt_update_sorted
       nên phần tử đầu tiên (index 0) luôn là tốt nhất nếu nó hợp lệ */
    if (table[0].addr != GR_ADDR_UNASSIGNED) {
        return &table[0];
    }
    
    return NULL;
}