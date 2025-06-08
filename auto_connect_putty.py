import time
import os
import sys
import subprocess
import serial.tools.list_ports

# -------------------------------
# CONFIGURABLE DEFAULTS
# -------------------------------
PUTTY_PATH = r"C:\Program Files\PuTTY\putty.exe"
BAUD_RATE = "115200"

def port_exists(name):
    ports = serial.tools.list_ports.comports()
    return any(name in p.device for p in ports)

def main():
    if len(sys.argv) < 2:
        print("Usage: python putty_reconnect.py COMx")
        sys.exit(1)

    port_name = "COM" + sys.argv[1].upper()

    print(f"Watching for {port_name}...")

    while True:
        if port_exists(port_name):
            print(f"Connecting to {port_name} with PuTTY...")
            subprocess.call([PUTTY_PATH, "-serial", port_name, "-sercfg", f"{BAUD_RATE},8,n,1,N"])
            print("PuTTY exited. Waiting for reconnect...")
        else:
            print(f"{port_name} not found. Waiting...")
        time.sleep(2)

if __name__ == "__main__":
    main()