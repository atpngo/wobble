import serial
import time


def current_ms():
    return round(time.time() * 1000)


tags = ["left_enc", "right_enc", "pitch", "yaw"]

monitor = serial.Serial("/dev/cu.usbserial-0001", 115200, timeout=1)

count = 0
with open("data.txt", "w+") as fout:
    try:
        while True:
            try:
                line = monitor.readline().decode("utf-8").strip()
                if not line:
                    continue
                count += 1
                print(f"{100*count/1000:.2f}%")
                fout.write(f"{current_ms()},{line}\n")
                fout.flush()
                if count >= 1000:
                    monitor.close()
                    fout.close()
                    break
            except Exception as e:
                # only catch non-fatal errors
                print(f"Read error: {e}")
                continue
    except KeyboardInterrupt:
        monitor.close()
        print("\nInterrupted by user, exiting...")
