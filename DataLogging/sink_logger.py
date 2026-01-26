import serial
import time
import csv
import datetime
import os
import sys

# ... (Giữ nguyên cấu hình PORT, BAUD_RATE ...)
SERIAL_PORT = 'COM18' 
BAUD_RATE = 115200    
LOG_DIR = r"C:\ncs\v3.0.1\zephyr\samples\GRADIENT-SRV\DataLogging"

# Dictionary để đếm số gói tin nhận được trong RAM
# Format: { '0x0002': 150, '0x0003': 200 }
rx_stats = {}

def get_log_filename():
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(LOG_DIR, f"wsn_pdr_full_{timestamp}.csv")

def main():
    print("--- WSN AUTOMATED PDR TESTER ---")
    filename = get_log_filename()
    
    if not os.path.exists(LOG_DIR): os.makedirs(LOG_DIR)
    
    try:
        with open(filename, mode='w', newline='', encoding='utf-8-sig') as file:
            writer = csv.writer(file)
            
            # HEADER MỞ RỘNG: Thêm các cột thống kê
            # Type: DATA (gói tin thường), EVENT (Start/Stop), RESULT (Kết quả PDR)
            headers = ["Timestamp", "Type", "SourceAddr", "SenderAddr", "Value_Seq_or_Tx", "RSSI", "Rx_Count_Measured", "PDR_Percent"]
            writer.writerow(headers)
            file.flush()
            
            print(f"Log file: {filename}")
            print("Ready. Press Button 3 on Sink to START test.")

            while True:
                try:
                    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                    ser.reset_input_buffer()

                    while True:
                        if ser.in_waiting > 0:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            
                            if "CSV_LOG" in line and "CSV_LOG," in line:
                                parts = line.split("CSV_LOG,")[1].split(',')
                                now = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                                
                                # Parse cơ bản
                                log_type = parts[0]
                                
                                # Xử lý từng loại log
                                if log_type == "DATA":
                                    # Format: DATA, src, sender, seq, rssi
                                    src = parts[1]
                                    sender = parts[2]
                                    seq = parts[3]
                                    rssi = parts[4]
                                    
                                    src_hex = f"0x{int(src):04x}"
                                    
                                    # 1. Tăng biến đếm trong RAM
                                    if src_hex not in rx_stats:
                                        rx_stats[src_hex] = 0
                                    rx_stats[src_hex] += 1
                                    
                                    # 2. Ghi log RAW
                                    writer.writerow([now, "DATA", src_hex, f"0x{int(sender):04x}", seq, rssi, "", ""])
                                    
                                    # In gọn để theo dõi
                                    # print(f"[{now}] RX Node {src_hex}: {seq} (Count: {rx_stats[src_hex]})")

                                elif log_type == "REPORT":
                                    # Format: REPORT, src, sender, total_tx, rssi
                                    src = parts[1]
                                    total_tx = int(parts[3])
                                    rssi = parts[4]
                                    src_hex = f"0x{int(src):04x}"
                                    
                                    # 3. Lấy số liệu đã đếm được
                                    measured_rx = rx_stats.get(src_hex, 0)
                                    
                                    # 4. TÍNH PDR
                                    if total_tx > 0:
                                        pdr = (measured_rx / total_tx) * 100.0
                                    else:
                                        pdr = 0.0
                                    
                                    # 5. Ghi dòng RESULT chứa TẤT CẢ thông số
                                    # Value cột 5 là TotalTx
                                    # Rx_Count_Measured là cột 7
                                    # PDR_Percent là cột 8
                                    writer.writerow([now, "RESULT", src_hex, f"0x{int(parts[2]):04x}", total_tx, rssi, measured_rx, f"{pdr:.2f}"])
                                    
                                    print(f"[{now}] >>> FINAL REPORT Node {src_hex} <<<")
                                    print(f"    - Tx Total (Node sent) : {total_tx}")
                                    print(f"    - Rx Count (Sink heard): {measured_rx}")
                                    print(f"    - PDR                  : {pdr:.2f} %")
                                    print("---------------------------------------------")

                                elif log_type == "EVENT":
                                    # Format: EVENT, TEST_START, Duration...
                                    # Reset bộ đếm khi bắt đầu test mới
                                    if "TEST_START" in parts[1]:
                                        rx_stats.clear()
                                        print(f"[{now}] --- NEW TEST SESSION STARTED ---")
                                    
                                    writer.writerow([now, "EVENT", parts[1], parts[2], "", "", "", ""])

                                file.flush()

                        else:
                            time.sleep(0.005)

                except Exception as e:
                    time.sleep(1)

    except KeyboardInterrupt:
        print("\nLogger Stopped.")

if __name__ == "__main__":
    main()