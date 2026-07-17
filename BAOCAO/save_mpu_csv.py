import serial
import csv
import time

PORT = "COM6"      # đổi đúng cổng ESP32 của bạn
BAUD = 115200

LABEL = "accident"  # đổi thành "accident" khi thu dữ liệu tai nạn

filename = f"mpu_{LABEL}.csv"

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(2)
ser.reset_input_buffer()

with open(filename, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["ax", "ay", "az", "accTotal", "accChange", "label"])

    print("Dang luu vao:", filename)
    print("Nhan Ctrl + C de dung")

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            if line.startswith("ax"):
                continue

            parts = line.split(",")

            if len(parts) == 5:
                ax, ay, az, accTotal, accChange = parts

                writer.writerow([ax, ay, az, accTotal, accChange, LABEL])
                f.flush()

                print("Da luu:", ax, ay, az, accTotal, accChange, LABEL)

        except KeyboardInterrupt:
            print("Da dung thu du lieu")
            break

        except Exception as e:
            print("Loi:", e)