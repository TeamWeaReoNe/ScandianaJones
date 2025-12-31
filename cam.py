import cv2
import os
import time
import piexif
from datetime import datetime
from pymavlink import mavutil

# --- Configuration ---
save2path = "/media/pi/INTENSO/"
TARGET_WIDTH = 1920
TARGET_HEIGHT = 1080
WARMUP_SECONDS = 2.0 
CAPTURE_RATE_FPS = 2.0  
MIN_TIME_BETWEEN_SHOTS = 1.0 / CAPTURE_RATE_FPS

# MAVLink Connection String (USB is usually /dev/ttyACM0 or /dev/ttyACM1)
MAVLINK_DEVICE = '/dev/ttyACM0' 
BAUD_RATE = 115200

# --- MAVLink Setup ---
print(f"Connecting to Pixhawk on {MAVLINK_DEVICE}...")
try:
    # autoreconnect=True helps if the USB is bumped
    master = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=BAUD_RATE, autoreconnect=True)
    # Wait for the first heartbeat to confirm connection
    master.wait_heartbeat()
    print("Pixhawk connected! Heartbeat received.")
except Exception as e:
    print(f"Failed to connect to Pixhawk: {e}")
    exit()

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

# --- Camera Setup ---
now = datetime.now()
folder_name = now.strftime("%Y-%m-%d_%H-%M-%S")
full_path = os.path.join(save2path, folder_name)

try:
    os.makedirs(full_path)
    print(f"Directory '{full_path}' created")
except OSError as e:
    print(f"Error creating directory: {e}")
    exit()

cam0 = cv2.VideoCapture(0)
cam1 = cv2.VideoCapture(2)

def set_camera_props(cam, width, height, cam_name):
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    actual_w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
    print(f"{cam_name} resolution set to: {int(actual_w)}")

set_camera_props(cam0, TARGET_WIDTH, TARGET_HEIGHT, "Cam0")
set_camera_props(cam1, TARGET_WIDTH, TARGET_HEIGHT, "Cam1")

# --- Warm-up Routine ---
print(f"Warming up cameras for {WARMUP_SECONDS} seconds...")
start_time = time.time()
while (time.time() - start_time) < WARMUP_SECONDS:
    cam0.read()
    cam1.read()
    # Also clear MAVLink buffer during warmup
    master.recv_match(type='GPS_RAW_INT', blocking=False)
    time.sleep(0.01)
print("Warm-up complete. Starting capture...")

# --- Main Loop ---
count = 0
last_save_time = 0

try:
    while True:
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
            current_alt = msg.alt / 1000.0 # Convert mm to meters
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

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cam0.release()
    cam1.release()
    cv2.destroyAllWindows()
    master.close()