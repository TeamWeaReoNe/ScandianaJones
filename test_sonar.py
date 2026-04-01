import sys
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import socket
import datetime
from dotenv import load_dotenv
import os


load_dotenv()

SAVE2PATH = os.getenv("SAVE2PATH")
FILE = os.getenv("FILE")
SONAR_BAUD_RATE = int(os.getenv("SONAR_BAUD_RATE"))
NUM_SAMPLES = int(os.getenv("NUM_SAMPLES"))
SPEED_OF_SOUND = int(os.getenv("SPEED_OF_SOUND"))
SAMPLE_TIME = float(os.getenv("SAMPLE_TIME"))
DEFAULT_LEVELS = tuple(os.getenv("DEFAULT_LEVELS"))
SONAR_DEVICE = os.getenv("SONAR_DEVICE")

SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2  # cm per row (0.99 cm per row)
PACKET_SIZE = 1 + 6 + 2 * NUM_SAMPLES + 1  # header + payload + checksum
MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION  # Total depth in cm


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
            print("⚠️ Checksum mismatch")
            continue

        # Unpack payload
        depth, temp_scaled, vDrv_scaled = struct.unpack(">HhH", payload[:6])
        depth = min(depth, NUM_SAMPLES)

        print(depth)
        print(temp_scaled)

        samples = struct.unpack(f">{NUM_SAMPLES}H", payload[6:])

        temperature = temp_scaled / 100.0
        drive_voltage = vDrv_scaled / 100.0
        values = np.array(samples)

        return values, depth, temperature, drive_voltage


def generate_dbt_sentence(depth_cm):
    depth_m = depth_cm / 100.0
    depth_ft = depth_m * 3.28084
    depth_fathoms = depth_m * 0.546807

    # Format the DBT sentence without checksum
    sentence_body = f"DBT,{depth_ft:.1f},f,{depth_m:.1f},M,{depth_fathoms:.1f},F"

    # Compute checksum
    checksum = 0
    for char in sentence_body:
        checksum ^= ord(char)

    nmea_sentence = f"${sentence_body}*{checksum:02X}"
    return nmea_sentence

def get_serial_ports():
    """Retrieve a list of available serial ports."""
    return [port.device for port in serial.tools.list_ports.comports()][::-1]

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except  Exception as e:
        print(e)
        return "127.0.0.1"

ip = get_local_ip()
#port = get_serial_ports()[1]
port = SONAR_DEVICE

print(f"IP: {ip}")
print(f"Serial ports: {port}")
print(f"Example nmea sentence: {generate_dbt_sentence(100)}")

csv_file = f"sonar_{FILE}"
LOG_FILE = os.path.join(SAVE2PATH, csv_file)
print(f"logfile {LOG_FILE}")

try:
    with serial.Serial(port, SONAR_BAUD_RATE, timeout=1) as ser:
        print("connected")
        while True:
            result = read_packet(ser)
            if result:
                values, depth, temperature, drive_voltage = result
                print(f"Depth: {depth}, Temp: {temperature}°C, Vdrv: {drive_voltage}V")
                with open(LOG_FILE, "a") as f:
                    data = f"{datetime.datetime.now()}, {result}\n"
                    f.write(data)
                    print(data)
                time.sleep(1)
            
except serial.SerialException as e:
    print(f"Serial Error: {e}")
