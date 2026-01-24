import serial
import time
import csv
import datetime
import os
import sys

# =============================================================================
# CẤU HÌNH (CONFIGURATION)
# =============================================================================

# Thay 'COM3' bằng cổng COM của nRF54 khi cắm vào máy tính
# Bạn có thể xem trong Device Manager -> Ports (COM & LPT)
SERIAL_PORT = 'COM18' 

BAUD_RATE = 115200    
TIMEOUT = 0.1          

LOG_DIR = r"C:\ncs\v3.0.1\zephyr\samples\GRADIENT-SRV\DataLogging"

# Tạo thư mục nếu chưa tồn tại
if not os.path.exists(LOG_DIR):
    os.makedirs(LOG_DIR)

# =============================================================================
# MAIN SCRIPT
# =============================================================================

def get_log_filename():
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(LOG_DIR, f"wsn_log_{timestamp}.csv")

def main():
    print("--- WSN DATA LOGGER FOR WINDOWS ---")
    print(f"Target Port: {SERIAL_PORT}")
    print(f"Baud Rate  : {BAUD_RATE}")
    
    filename = get_log_filename()
    
    try:
        # Windows dùng encoding 'utf-8-sig' để mở trực tiếp bằng Excel không lỗi font
        with open(filename, mode='w', newline='', encoding='utf-8-sig') as file:
            writer = csv.writer(file)
            
            # Ghi Header
            headers = ["Timestamp", "SourceAddr", "SenderAddr", "Payload", "RSSI", "HopCount"]
            writer.writerow(headers)
            file.flush()
            
            print(f"Đang lưu tại: {filename}")
            print("Đang quét dữ liệu... (Nhấn Ctrl+C để dừng)")

            while True:
                try:
                    # Mở cổng Serial
                    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
                    print(f"KẾT NỐI THÀNH CÔNG: {SERIAL_PORT}")
                    
                    ser.reset_input_buffer()

                    while True:
                        if ser.in_waiting > 0:
                            try:
                                # Đọc dữ liệu từ nRF54
                                line = ser.readline().decode('utf-8', errors='ignore').strip()
                                
                                # Lọc dòng chứa từ khóa CSV_LOG
                                if "CSV_LOG" in line:
                                    # Cắt bỏ ký tự rác nếu có
                                    start_idx = line.find("CSV_LOG")
                                    clean_data = line[start_idx:]
                                    parts = clean_data.split(',')
                                    
                                    if len(parts) >= 6:
                                        now = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                                        
                                        # Định dạng lại địa chỉ sang Hex cho dễ đọc
                                        src_hex = f"0x{int(parts[1]):04x}"
                                        snd_hex = f"0x{int(parts[2]):04x}"
                                        
                                        row_data = [now, src_hex, snd_hex, parts[3], parts[4], parts[5]]
                                        
                                        # Ghi vào file
                                        writer.writerow(row_data)
                                        file.flush() # Lưu ngay lập tức
                                        
                                        print(f"[{now}] Nhận từ {src_hex}: Data={parts[3]} RSSI={parts[4]}")
                                        
                            except Exception as e:
                                continue
                        else:
                            time.sleep(0.01)

                except serial.SerialException:
                    print(f"LỖI: Không tìm thấy {SERIAL_PORT}. Đang thử lại sau 2s...")
                    time.sleep(2)
                except KeyboardInterrupt:
                    raise

    except KeyboardInterrupt:
        print("\n--- Đã dừng Logger ---")
    except Exception as e:
        print(f"\n--- Lỗi hệ thống: {e} ---")
    finally:
        print(f"Dữ liệu đã được lưu an toàn tại: {filename}")
        input("Nhấn Enter để thoát...")

if __name__ == "__main__":
    main()