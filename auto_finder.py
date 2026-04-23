import socket
import concurrent.futures

def check_port(ip, port):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.2)
        if s.connect_ex((ip, port)) == 0:
            return ip
    except:
        pass
    finally:
        s.close()
    return None

ips = [f"192.168.1.{i}" for i in range(1, 255)]

print("Scanning for ESP32-CAM (Port 81)...")
with concurrent.futures.ThreadPoolExecutor(max_workers=50) as e:
    cam_ips = [ip for ip in e.map(lambda ip: check_port(ip, 81), ips) if ip]

print("Scanning for Pico (Port 8765)...")
with concurrent.futures.ThreadPoolExecutor(max_workers=50) as e:
    pico_ips = [ip for ip in e.map(lambda ip: check_port(ip, 8765), ips) if ip]

if cam_ips: print(f"Found Camera at: {cam_ips[0]}")
else: print("Camera NOT found.")

if pico_ips: print(f"Found Pico at: {pico_ips[0]}")
else: print("Pico NOT found.")
