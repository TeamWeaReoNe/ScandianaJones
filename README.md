# Beschreibung

Der Python Code in diesem Repository dient zum Verarbeiten der Messdaten, die mittels des autonomen Bootes **Scandiana Jones** gesammelt werden.

Der Code wurde getestet mit Python 3.13.5. Es wird empfohlen, ein virtuelles Environment anzulegen. 

Verwendete Hardware:

| Komponente | Beschreibung |
|---|---|
| Raspberry Pi 4 Model B 2 GB| Messdatenverarbeitung mit Python |
| Open Echo Sonar Shield for Arduino | siehe https://github.com/Neumi/open_echo |
| Arduino Uno  | für Open Echo Shield, connected to Raspberry via USB |
| Pixhawk Controller | für GPS Daten, connected to Raspberry via USB |
| zwei USB Kamaras (Webcam) | Erzeugung der Bilddaten |
| USB Stick | zum Speichern der Messdaten |

## Vorbereitung

Es wird empfohlen, ein virtuelles Environment zu erzeugen und die benötigten Module zu installieren:

```python
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Dateien

| file | description |
|---|---|
| .env | contains the environment variables |
| requirements.txt | required Python modules for pip install |
| scandiana.py | main program for aquire and save measuring date |
| test_mavlink.py | test program for GPS MAVLINK interface |
| test_sonar.py | test program for Open Echo interface |


## Environment

Das Projekt enthält eine **.env** Datei. Diese enthält alle notwendigen Steuerungsdaten wie z.B.

* Informationen zu den Schnittstellen
* Baudraten
* Speicherorte

Diese Datei muss an die jeweilige Umgebung angepasst werden.

Beispiel:

```
SAVE2PATH=/media/pi/INTENSO/
FILE=data.csv
MAVLINK_DEVICE=/dev/ttyACM0
MAVLINK_BAUD_RATE=115200
SONAR_DEVICE=/dev/ttyACM1
SONAR_BAUD_RATE=250000
NUM_SAMPLES=1800
SPEED_OF_SOUND=1450
SAMPLE_TIME=13.2e-6
DEFAULT_LEVELS=(0, 256)
```

## Starten des Messvorgangs

Der Messvorgang wird mit 

`python scandiana.py`

gestartet. Im Verzeichnis **SAVE2PATH** wird ein Unterordner mit Datum & Zeit angelegt. In diesem werden die aufgenommenen Bilder mit EXIF-Daten und eine CSV-Datei mit den Messdaten gespeichert.