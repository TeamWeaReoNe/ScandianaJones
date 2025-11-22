import socket
import pynmea2
from dotenv import load_dotenv
import os
import datetime

load_dotenv()

FILE = os.getenv("FILE")
HOST = os.getenv("HOST")
PORT = int(os.getenv("PORT"))

def read_nmea_stream(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        sock_file = sock.makefile('r', newline='\r\n')  

        for line in sock_file:
            line = line.strip()
            with open(FILE, "a") as f:
                data = f"{datetime.datetime.now()}, {line}\n"
                f.write(data)
                print(data)

if __name__ == '__main__':
    read_nmea_stream(HOST, PORT)