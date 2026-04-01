import cv2
import os
import time
import piexif
from datetime import datetime
from pymavlink import mavutil
from dotenv import load_dotenv
import serial
import struct
import numpy as np


# --- Configuration ---

# load configuration from the .env file
load_dotenv()

SAVE2PATH = os.getenv("SAVE2PATH")
FILE = os.getenv("FILE")

SONAR_BAUD_RATE = int(os.getenv("SONAR_BAUD_RATE"))
NUM_SAMPLES = int(os.getenv("NUM_SAMPLES"))
SPEED_OF_SOUND = int(os.getenv("SPEED_OF_SOUND"))
SAMPLE_TIME = float(os.getenv("SAMPLE_TIME"))
DEFAULT_LEVELS = tuple(os.getenv("DEFAULT_LEVELS"))

MAVLINK_DEVICE = os.getenv("MAVLINK_DEVICE") 
MAVLINK_BAUD_RATE = os.getenv("MAVLINK_BAUD_RATE")


TARGET_WIDTH = 1920
TARGET_HEIGHT = 1080
WARMUP_SECONDS = 2.0 
CAPTURE_RATE_FPS = 2.0  
MIN_TIME_BETWEEN_SHOTS = 1.0 / CAPTURE_RATE_FPS

# config for sonar (openecho)

SONAR_BAUD_RATE = int(os.getenv("SONAR_BAUD_RATE"))
NUM_SAMPLES = int(os.getenv("NUM_SAMPLES"))
SPEED_OF_SOUND = int(os.getenv("SPEED_OF_SOUND"))
SAMPLE_TIME = float(os.getenv("SAMPLE_TIME"))
DEFAULT_LEVELS = tuple(os.getenv("DEFAULT_LEVELS"))
SONAR_DEVICE = os.getenv("SONAR_DEVICE")

SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2  # cm per row (0.99 cm per row)
PACKET_SIZE = 1 + 6 + 2 * NUM_SAMPLES + 1  # header + payload + checksum
MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION  # Total depth in cm

# read the sonar data
# this code comes from https://github.com/Neumi/open_echo

def read_packet(ser):
    while True:
        header = ser.read(1)
        if header != b'\xAA':
            continue  # Wait for the start byte

        payload = ser.read(6 + 2 * NUM_SAMPLES)
        checksum = ser.read(1)

        if len(payload) != 6 + 2 * NUM_SAMPLES or len(checksum) != 1:
            continue  # Incomplete packet

        # Verify checksum
        calc_checksum = 0
        for byte in payload:
            calc_checksum ^= byte
        if calc_checksum != checksum[0]:
            print("?? Checksum mismatch")
            continue

        # Unpack payload
        depth, temp_scaled, vDrv_scaled = struct.unpack(">HhH", payload[:6])
        depth = min(depth, NUM_SAMPLES)

        #print(depth)
        #print(temp_scaled)

        samples = struct.unpack(f">{NUM_SAMPLES}H", payload[6:])

        temperature = temp_scaled / 100.0
        drive_voltage = vDrv_scaled / 100.0
        values = np.array(samples)

        return values, depth, temperature, drive_voltage



# Global variables to store the latest GPS data
current_lat = 0.0
current_lon = 0.0
current_alt = 0.0
gps_fix_type = 0 # 0-1: No Fix, 2: 2D, 3: 3D

# --- GPS Helper Functions ---
def to_deg(value, loc):
    """Converts decimal coordinates to EXIF-ready Degrees/Minutes/Seconds."""
    if value < 0:
        loc_value = loc[1]
    else:
        loc_value = loc[0]
    
    abs_value = abs(value)
    deg = int(abs_value)
    t1 = (deg, 1)
    
    min_val = (abs_value - deg) * 60
    min_int = int(min_val)
    t2 = (min_int, 1)
    
    sec_val = (min_val - min_int) * 60
    sec_int = int(sec_val * 10000)
    t3 = (sec_int, 10000)
    
    return (t1, t2, t3), loc_value

def create_gps_bytes(lat, lon, alt):
    """Creates EXIF GPS bytes dynamically."""
    lat_deg, lat_ref = to_deg(lat, ["N", "S"])
    lon_deg, lon_ref = to_deg(lon, ["E", "W"])
    
    # Altitude handling (reference 0 = above sea level)
    alt_ref = 0 if alt >= 0 else 1
    alt_rational = (int(abs(alt) * 100), 100)

    gps_ifd = {
        piexif.GPSIFD.GPSLatitudeRef: lat_ref,
        piexif.GPSIFD.GPSLatitude: lat_deg,
        piexif.GPSIFD.GPSLongitudeRef: lon_ref,
        piexif.GPSIFD.GPSLongitude: lon_deg,
        piexif.GPSIFD.GPSAltitudeRef: alt_ref,
        piexif.GPSIFD.GPSAltitude: alt_rational,
    }
    
    exif_dict = {"GPS": gps_ifd}
    return piexif.dump(exif_dict)



def set_camera_props(cam, width, height, cam_name):
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    actual_w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
    print(f"... {cam_name} resolution set to: {int(actual_w)}")



# -- main program

# sonar setup

try:
  print("trying to connect to sonar (openecho) ...")
  print(f"... port {SONAR_DEVICE} with baudrate {SONAR_BAUD_RATE} ...")
  sonar_serial = serial.Serial(SONAR_DEVICE, SONAR_BAUD_RATE, timeout=1)
except Exception as e:
    print(f"... Failed to connect to sonar: {e}")
    exit()
print("... connected to sonar")

# --- MAVLink Setup ---
print(f"Connecting to Pixhawk via MAVLINK on {MAVLINK_DEVICE}...")
try:
    # autoreconnect=True helps if the USB is bumped
    master = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD_RATE, autoreconnect=True)
    # Wait for the first heartbeat to confirm connection
    master.wait_heartbeat()
    print("... Pixhawk connected! Heartbeat received.")
except Exception as e:
    print(f"... Failed to connect to Pixhawk: {e}")
    exit()

# --- Camera setup ---

print("setting up cameras ...")
cam0 = cv2.VideoCapture(0)
cam1 = cv2.VideoCapture(2)

set_camera_props(cam0, TARGET_WIDTH, TARGET_HEIGHT, "Cam0")
set_camera_props(cam1, TARGET_WIDTH, TARGET_HEIGHT, "Cam1")

# --- Warm-up Routine ---
print(f"... Warming up cameras for {WARMUP_SECONDS} seconds...")
start_time = time.time()
while (time.time() - start_time) < WARMUP_SECONDS:
    cam0.read()
    cam1.read()
    # Also clear MAVLink buffer during warmup (crash sometimes?)
    master.recv_match(type='GPS_RAW_INT', blocking=False)
    time.sleep(0.01)
print("Setting up and Warm-up complete")

# --- File Setup ---
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

try:
    csv_file = os.path.join(full_path,FILE)
    print(f"... logging to: {csv_file}")
    file_handle = open(csv_file, "a")
    csv_header = "date-time,lattitude,longitude,depth\n"
    file_handle.write(csv_header)
except OSError as e:
    print(f"... Error open logging file: {e}")
    exit()

# --- Main Loop ---
print("Starting capture ...")
count = 0
last_save_time = 0

try:
    while True:
        # 0. try to get depth from sonar
        result = read_packet(sonar_serial)
        if result:
          values, depth, temperature, drive_voltage = result
          print(f"... Depth: {depth}")
        # 1. Update Camera (Clear buffer)
        ret0, frame0 = cam0.read()
        ret1, frame1 = cam1.read()
        
        # 2. Update GPS Data
        # We read messages without blocking. 
        # GPS_RAW_INT provides raw sensor data (lat/lon in degE7, alt in mm)
        msg = master.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg:
            # Convert integers (degE7) to float degrees
            current_lat = msg.lat / 1.0e7
            current_lon = msg.lon / 1.0e7
            current_alt = depth # this is the first test, maybe we have to adjust the value
            #current_alt = msg.alt / 1000.0 # Convert mm to meters
            gps_fix_type = msg.fix_type # 3 = 3D Fix

        current_time = time.time()

        # 3. Check Timer
        if (current_time - last_save_time) >= MIN_TIME_BETWEEN_SHOTS:
            
            # Generate EXIF data for this specific moment
            # (If fix is bad, these might be 0.0, but we save them anyway)
            gps_bytes = create_gps_bytes(current_lat, current_lon, current_alt)
            
            saved_any = False
            
            if ret0:
                filename0 = os.path.join(full_path, f"webcam0_{count}.jpg")
                cv2.imwrite(filename0, frame0)
                try:
                    piexif.insert(gps_bytes, filename0)
                except Exception:
                    pass # Fail silently if EXIF write fails to keep loop running
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
                fix_str = "3D Fix" if gps_fix_type == 3 else f"FixType: {gps_fix_type}"
                print(f"Saved pair {count} | GPS: {current_lat:.6f}, {current_lon:.6f} ({fix_str})")
                count += 1
                last_save_time = current_time
        
        # 4. save the data to the csv file
        csv_data = f"{datetime.fromtimestamp(current_time)},{current_lat},{current_lon},{depth}\n"
        file_handle.write(csv_data)
        
        # this doesn't work at the moment
        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

finally:
    file_handle.close()
    cam0.release()
    cam1.release()
    cv2.destroyAllWindows()
    master.close()