# Full version: original prints restored + minimal threading

import cv2
import os
import time
import piexif
import threading
from datetime import datetime
from pymavlink import mavutil
from dotenv import load_dotenv
import serial
import struct
import numpy as np

# --- Configuration ---
load_dotenv()

SAVE2PATH = os.getenv("SAVE2PATH")
FILE = os.getenv("FILE")

SONAR_BAUD_RATE = int(os.getenv("SONAR_BAUD_RATE"))
NUM_SAMPLES = int(os.getenv("NUM_SAMPLES"))
SPEED_OF_SOUND = int(os.getenv("SPEED_OF_SOUND"))
SAMPLE_TIME = float(os.getenv("SAMPLE_TIME"))
DEFAULT_LEVELS = tuple(os.getenv("DEFAULT_LEVELS"))

MAVLINK_DEVICE = os.getenv("MAVLINK_DEVICE") 
MAVLINK_BAUD_RATE = int(os.getenv("MAVLINK_BAUD_RATE"))

SONAR_DEVICE = os.getenv("SONAR_DEVICE")

TARGET_WIDTH = 1920
TARGET_HEIGHT = 1080
CAPTURE_RATE_FPS = 5.0  # increased capture frequency (smaller interval)  
MIN_TIME_BETWEEN_SHOTS = 1.0 / CAPTURE_RATE_FPS

# Shared variables
current_lat = 0.0
current_lon = 0.0
current_alt = 0.0
gps_fix_type = 0
depth = 0

lock = threading.Lock()
running = True

# --- ORIGINAL read_packet ---

def read_packet(ser):
    while True:
        header = ser.read(1)
        if header != b"\xaa":
            continue

        payload = ser.read(6 + NUM_SAMPLES)
        checksum = ser.read(1)

        if len(payload) != 6 + NUM_SAMPLES or len(checksum) != 1:
            continue

        calc_checksum = 0
        for byte in payload:
            calc_checksum ^= byte
        if calc_checksum != checksum[0]:
            print(f"!! Checksum mismatch: {calc_checksum} != {checksum[0]}")
            continue

        depth_val, temp_scaled, vDrv_scaled = struct.unpack("<HhH", payload[:6])
        depth_val = min(depth_val, NUM_SAMPLES)

        return depth_val

# --- SONAR THREAD ---

def sonar_loop():
    global depth

    try:
        print("trying to connect to sonar (openecho) ...")
        print(f"... port {SONAR_DEVICE} with baudrate {SONAR_BAUD_RATE} ...")
        ser = serial.Serial(SONAR_DEVICE, SONAR_BAUD_RATE, timeout=1)
        print("... connected to sonar")
    except Exception as e:
        print(f"... Failed to connect to sonar: {e}")
        return

    while running:
        result = read_packet(ser)
        if result is not None:
            with lock:
                depth = result
            # Depth print moved to camera loop to match original behavior

# --- MAVLINK THREAD ---

def mavlink_loop():
    global current_lat, current_lon, current_alt, gps_fix_type

    print(f"Connecting to Pixhawk via MAVLINK on {MAVLINK_DEVICE}...")
    try:
        master = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD_RATE, autoreconnect=True)
        master.wait_heartbeat()
        print("... Pixhawk connected! Heartbeat received.")
    except Exception as e:
        print(f"... Failed to connect to Pixhawk: {e}")
        return

    while running:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg:
            with lock:
                current_lat = msg.lat / 1.0e7
                current_lon = msg.lon / 1.0e7
                current_alt = depth
                gps_fix_type = msg.fix_type

# --- CAMERA THREAD ---

def camera_loop(full_path):
    global current_lat, current_lon, depth

    print("setting up cameras ...")
    cam0 = cv2.VideoCapture(0)
    cam1 = cv2.VideoCapture(2)

    def set_camera_props(cam, name):
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_WIDTH)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        print(f"... {name} resolution set to: {int(actual_w)}")

    set_camera_props(cam0, "Cam0")
    set_camera_props(cam1, "Cam1")

    WARMUP_SECONDS = 2.0
    print(f"... Warming up cameras for {WARMUP_SECONDS} seconds...")
    start_time = time.time()
    while (time.time() - start_time) < WARMUP_SECONDS:
        cam0.read()
        cam1.read()
        time.sleep(0.01)
    print("Setting up and Warm-up complete")

    print("Starting capture ...")
    last_save_time = 0
    count = 0

    while running:
        ret0, frame0 = cam0.read()
        ret1, frame1 = cam1.read()

        current_time = time.time()

        # CSV logging (same behavior as original)
        with lock:
            lat_csv = current_lat
            lon_csv = current_lon
            depth_csv = depth

        csv_data = f"{datetime.fromtimestamp(current_time)},{lat_csv},{lon_csv},{depth_csv}\n"
        file_handle.write(csv_data)
        file_handle.flush()  # ensure each row is written immediately

        if (current_time - last_save_time) >= MIN_TIME_BETWEEN_SHOTS:
            with lock:
                lat = current_lat
                lon = current_lon
                d = depth
                fix = gps_fix_type

            gps_bytes = create_gps_bytes(lat, lon, d)

            saved_any = False

            if ret0:
                filename0 = os.path.join(full_path, f"webcam0_{count}.jpg")
                cv2.imwrite(filename0, frame0)
                try:
                    piexif.insert(gps_bytes, filename0)
                except Exception:
                    pass
                saved_any = True

            if ret1:
                filename1 = os.path.join(full_path, f"webcam1_{count}.jpg")
                cv2.imwrite(filename1, frame1)
                try:
                    piexif.insert(gps_bytes, filename1)
                except Exception:
                    pass
                saved_any = True

            if saved_any:
                fix_str = "3D Fix" if fix == 3 else f"FixType: {fix}"
                print(f"... Depth: {d}")
                print(f"Saved pair {count} | GPS: {lat:.6f}, {lon:.6f} ({fix_str})")
                count += 1
                last_save_time = current_time

        time.sleep(0.005)

# --- GPS EXIF ---

def to_deg(value, loc):
    if value < 0:
        loc_value = loc[1]
    else:
        loc_value = loc[0]

    abs_value = abs(value)
    deg = int(abs_value)
    min_val = (abs_value - deg) * 60
    min_int = int(min_val)
    sec_val = (min_val - min_int) * 60

    return ((deg,1),(min_int,1),(int(sec_val*10000),10000)), loc_value


def create_gps_bytes(lat, lon, alt):
    lat_deg, lat_ref = to_deg(lat, ["N", "S"])
    lon_deg, lon_ref = to_deg(lon, ["E", "W"])

    gps_ifd = {
        piexif.GPSIFD.GPSLatitudeRef: lat_ref,
        piexif.GPSIFD.GPSLatitude: lat_deg,
        piexif.GPSIFD.GPSLongitudeRef: lon_ref,
        piexif.GPSIFD.GPSLongitude: lon_deg,
        piexif.GPSIFD.GPSAltitude: (int(abs(alt)*100),100)
    }

    return piexif.dump({"GPS": gps_ifd})


# --- MAIN ---

print("setting up save path ...")
now = datetime.now()
folder_name = now.strftime("%Y-%m-%d_%H-%M-%S")
full_path = os.path.join(SAVE2PATH, folder_name)

try:
    os.makedirs(full_path)
    print(f"... Directory '{full_path}' created")
except OSError as e:
    print(f"... Error creating directory: {e}")
    exit()

# --- CSV SETUP ---

csv_file = os.path.join(full_path, FILE)
print(f"... logging to: {csv_file}")
try:
    file_handle = open(csv_file, "a")
    csv_header = "date-time,lattitude,longitude,depth\n"
    file_handle.write(csv_header)
except OSError as e:
    print(f"... Error open logging file: {e}")
    exit()


threads = [
    threading.Thread(target=sonar_loop),
    threading.Thread(target=mavlink_loop),
    threading.Thread(target=camera_loop, args=(full_path,))
]

for t in threads:
    t.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    running = False

for t in threads:
    t.join()

file_handle.close()

