import serial
import time
import csv
import datetime
import os
import sys

# --- CẤU HÌNH ---
SERIAL_PORT = 'COM18' 
BAUD_RATE = 115200      
LOG_DIR = r"C:\ncs\v3.0.1\zephyr\samples\GRADIENT-SRV\DataLogging"

# Dictionary để đếm số gói tin nhận được trong RAM để tính PDR tức thời
rx_stats = {}

def get_log_filename():
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(LOG_DIR, f"wsn_pdr_hops_{timestamp}.csv")

def main():
    print("--- WSN AUTOMATED PDR & HOPCOUNT LOGGER ---")
    filename = get_log_filename()
    
    if not os.path.exists(LOG_DIR): 
        os.makedirs(LOG_DIR)
    
    try:
        with open(filename, mode='w', newline='', encoding='utf-8-sig') as file:
            writer = csv.writer(file)
            
            # HEADER MỞ RỘNG: Thêm cột HopCount
            # Cột: 1.Timestamp, 2.Type, 3.Source, 4.Sender, 5.Value/Seq, 6.RSSI, 7.HopCount, 8.Rx_Measured, 9.PDR%
            headers = ["Timestamp", "Type", "SourceAddr", "SenderAddr", "Value_Seq_or_Tx", "RSSI", "HopCount", "Rx_Count_Measured", "PDR_Percent"]
            writer.writerow(headers)
            file.flush()
            
            print(f"Log file: {filename}")
            print("Ready. Press Button 3 on Sink to START test.")

            while True:
                try:
                    # Kết nối Serial
                    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                    ser.reset_input_buffer()

                    while True:
                        if ser.in_waiting > 0:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            
                            if "CSV_LOG" in line and "CSV_LOG," in line:
                                # Tách bỏ tiền tố CSV_LOG, và lấy các phần tử
                                raw_data = line.split("CSV_LOG,")[1]
                                parts = raw_data.split(',')
                                now = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                                
                                log_type = parts[0]
                                
                                # --- XỬ LÝ GÓI TIN DATA ---
                                if log_type == "DATA":
                                    # FW format: DATA, src, sender, seq, rssi, hops
                                    src = parts[1]
                                    sender = parts[2]
                                    seq = parts[3]
                                    rssi = parts[4]
                                    # Đọc HopCount nếu có (mặc định là 0 nếu firmware cũ)
                                    hops = parts[5] if len(parts) >= 6 else "0"
                                    
                                    src_hex = f"0x{int(src):04x}"
                                    sender_hex = f"0x{int(sender):04x}"
                                    
                                    # Tăng biến đếm trong RAM
                                    rx_stats[src_hex] = rx_stats.get(src_hex, 0) + 1
                                    
                                    # Ghi dòng DATA
                                    # Cột: Time, Type, Src, Snd, Seq, RSSI, Hop, Rx_M, PDR
                                    writer.writerow([now, "DATA", src_hex, sender_hex, seq, rssi, hops, "", ""])
                                    
                                    print(f"[{now}] DATA from {src_hex} via {sender_hex} | Seq: {seq} | RSSI: {rssi} | Hops: {hops}")

                                # --- XỬ LÝ GÓI TIN REPORT (KẾT THÚC) ---
                                elif log_type == "REPORT":
                                    # FW format: REPORT, src, sender, total_tx, rssi
                                    src = parts[1]
                                    sender_hex = f"0x{int(parts[2]):04x}"
                                    total_tx = int(parts[3])
                                    rssi = parts[4]
                                    src_hex = f"0x{int(src):04x}"
                                    
                                    measured_rx = rx_stats.get(src_hex, 0)
                                    pdr = (measured_rx / total_tx * 100.0) if total_tx > 0 else 0.0
                                    
                                    # Ghi dòng RESULT
                                    writer.writerow([now, "RESULT", src_hex, sender_hex, total_tx, rssi, "", measured_rx, f"{pdr:.2f}"])
                                    
                                    print(f"\n[{now}] >>> FINAL REPORT Node {src_hex} <<<")
                                    print(f"    - Packets Sent (Tx): {total_tx}")
                                    print(f"    - Packets Received (Rx): {measured_rx}")
                                    print(f"    - PDR: {pdr:.2f} %")
                                    print("---------------------------------------------")

                                # --- XỬ LÝ SỰ KIỆN HỆ THỐNG ---
                                elif log_type == "EVENT":
                                    event_name = parts[1]
                                    msg = parts[2] if len(parts) >= 3 else ""
                                    
                                    # Reset bộ đếm khi bắt đầu phiên test mới
                                    if "TEST_START" in event_name:
                                        rx_stats.clear()
                                        print(f"\n[{now}] --- TEST SESSION STARTED ---")
                                    
                                    writer.writerow([now, "EVENT", event_name, msg, "", "", "", "", ""])

                                file.flush()

                        else:
                            time.sleep(0.005)

                except serial.SerialException:
                    print("Waiting for Serial Port...")
                    time.sleep(1)
                except Exception as e:
                    print(f"Error: {e}")
                    time.sleep(1)

    except KeyboardInterrupt:
        print("\nLogger Stopped by User.")

if __name__ == "__main__":
    main()