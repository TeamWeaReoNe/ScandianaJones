#!/usr/bin/env python3
from pymavlink import mavutil
import time

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'   # Change to your USB port (e.g., COM3 on Windows)
BAUD_RATE = 57600              # Common baud rate for flight controllers
LOG_FILE = 'gps_log.csv'

# --- Connect to flight controller ---
print(f"Connecting to flight controller on {SERIAL_PORT} at {BAUD_RATE} baud...")
master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)

# Wait for heartbeat to confirm connection
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system: {master.target_system}, component: {master.target_component}")

# --- Prepare log file ---
with open(LOG_FILE, 'w') as f:
    f.write("timestamp,lat,lon,alt_m,fix_type,satellites\n")

print(f"Logging GPS data to {LOG_FILE}... Press Ctrl+C to stop.")

try:
    while True:
        # Receive GPS_RAW_INT message
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg:
            timestamp = time.time()
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0  # altitude in meters
            fix_type = msg.fix_type
            satellites = msg.satellites_visible

            # Print to console
            print(f"Time: {timestamp:.2f}, Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.2f} m, Fix: {fix_type}, Sats: {satellites}")

            # Append to log file
            with open(LOG_FILE, 'a') as f:
                f.write(f"{timestamp},{lat},{lon},{alt},{fix_type},{satellites}\n")

except KeyboardInterrupt:
    print("\nLogging stopped by user.")

#Provide a full working Python script for reading GPS data and logging it to a file using an USB connection between controller and host
