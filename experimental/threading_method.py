import serial
import numpy as np
from scipy.optimize import least_squares
from pyproj import Transformer
import pynmea2
from datetime import datetime, timezone
import threading
import queue
import time

"""
Experimental Positioning System: Threading Approach

Description:
    This script separates the UWB reading (Input) and NMEA writing (Output) 
    tasks into two parallel threads communicating via a FIFO Queue.
    
    Goal: To prevent the slow NMEA writing process from blocking the high-speed 
    UWB data reading.
    
    Outcome: While effective for separation, Python's Global Interpreter Lock (GIL)
    and queue management introduced slight latency compared to the synchronous 
    approach in 'main_standalone.py'. Kept for architectural reference.
"""

# --- Configuration ---
UWB_PORT = 'COM5'
UWB_BAUD = 115200
PIXHAWK_PORT = 'COM3'
PIXHAWK_BAUD = 38400

MAX_VALID_DISTANCE = 100
OUTLIER_THRESHOLD = 5
DRONE_HEIGHT = 0.0

# Fixed Anchors for Testing
ANCHOR_GPS_POINTS = [
    (39.872126, 32.732462),
    (39.872081, 32.732290),
    (39.872046, 32.732461)
]

# Thread-Safe Queue
data_queue = queue.Queue()
previous_uwb_measurements = None

# --- Coordinate Helpers ---

def local_to_gps(x, y, reference_gps):
    transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)
    transformer_to_latlon = Transformer.from_crs("EPSG:32636", "EPSG:4326", always_xy=True)
    ref_lat, ref_lon = reference_gps
    x0, y0 = transformer_to_utm.transform(ref_lon, ref_lat)
    utm_x, utm_y = x0 + x, y0 + y
    lon, lat = transformer_to_latlon.transform(utm_x, utm_y)
    return lat, lon

def gps_to_local(points):
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)
    utm_points = [transformer.transform(lon, lat) for lat, lon in points]
    x0, y0 = utm_points[0]
    return [(x - x0, y - y0) for x, y in utm_points]

# Pre-calc geometry
local_system = gps_to_local(ANCHOR_GPS_POINTS)
local_coordinates = [[x, y] for x, y in local_system]
reference_gps = [ANCHOR_GPS_POINTS[0][0], ANCHOR_GPS_POINTS[0][1]]

def cost_function(p, uwb_measurements, anchor_coords):
    residuals = []
    for d_meas, p_anchor in zip(uwb_measurements, anchor_coords):
        d_calc = np.linalg.norm(np.array(p) - p_anchor)
        residuals.append((d_calc - d_meas)**2)
    return residuals

def decimal_to_nmea(coord: float, is_lat: bool = True):
    degrees = int(abs(coord))
    minutes = (abs(coord) - degrees) * 60
    deg_width = 2 if is_lat else 3
    deg_str = f"{degrees:0{deg_width}d}"
    min_str = f"{minutes:07.4f}"
    direction = ('N' if coord >= 0 else 'S') if is_lat else ('E' if coord >= 0 else 'W')
    return deg_str + min_str, direction

# --- Thread 1: UWB Reader (Producer) ---
def uwb_reader():
    global previous_uwb_measurements
    try:
        with serial.Serial(UWB_PORT, UWB_BAUD, timeout=0.1) as ser:
            print(f"[Thread-1] UWB Connected on {UWB_PORT}")
            while True:
                try:
                    data = ser.readline().decode('utf-8').strip()
                    if not data: continue
                    
                    values = [float(val.strip()) for val in data.split(',')]
                    if len(values) < 3: continue
                    
                    if any(v <= 0 or v > MAX_VALID_DISTANCE for v in values): continue

                    # Outlier Filter
                    if previous_uwb_measurements is not None:
                        deltas = [abs(a - b) for a, b in zip(values, previous_uwb_measurements)]
                        if any(delta > OUTLIER_THRESHOLD for delta in deltas):
                            continue

                    previous_uwb_measurements = values.copy()

                    # Calculate Position immediately
                    uwb_measurements_2d = [np.sqrt(max(d**2 - DRONE_HEIGHT**2, 0)) for d in values]
                    result = least_squares(cost_function, [0, 0], args=(uwb_measurements_2d, local_coordinates))
                    x, y = result.x
                    lat, lon = local_to_gps(x, y, reference_gps)

                    # Put result in Queue
                    data_queue.put((values, x, y, lat, lon))

                except Exception as e:
                    print(f"[Reader Error] {e}")
    except serial.SerialException:
        print("[Thread-1] Failed to connect.")

# --- Thread 2: NMEA Writer (Consumer) ---
def nmea_writer():
    try:
        with serial.Serial(PIXHAWK_PORT, PIXHAWK_BAUD, timeout=0.1) as ser2:
            print(f"[Thread-2] Pixhawk Connected on {PIXHAWK_PORT}")
            while True:
                try:
                    # Blocks until data is available
                    uwb_values, x, y, lat, lon = data_queue.get()

                    dt = datetime.now(timezone.utc)
                    time_str = dt.strftime("%H%M%S")
                    date_str = dt.strftime("%d%m%y")

                    lat_nmea, lat_dir = decimal_to_nmea(lat, True)
                    lon_nmea, lon_dir = decimal_to_nmea(lon, False)

                    # Construct NMEA
                    rmc = pynmea2.RMC('GP','RMC', (
                        time_str, 'A', lat_nmea, lat_dir, lon_nmea, lon_dir,
                        '0.0','0.0', date_str, '', '', 'A'
                    ))
                    gga = pynmea2.GGA('GP','GGA', (
                        time_str, lat_nmea, lat_dir, lon_nmea, lon_dir,
                        '1','10','0.9', f'{DRONE_HEIGHT:.1f}','M','0.0','M','',''
                    ))

                    for s in (rmc, gga):
                        line = str(s) + '\r\n'
                        ser2.write(line.encode('ascii'))
                    
                    print(f"[NMEA] Lat: {lat:.6f}, Lon: {lon:.6f}")
                    data_queue.task_done()

                except Exception as e:
                    print(f"[Writer Error] {e}")
    except serial.SerialException:
        print("[Thread-2] Failed to connect.")

# --- Main ---
if __name__ == "__main__":
    t1 = threading.Thread(target=uwb_reader, daemon=True)
    t2 = threading.Thread(target=nmea_writer, daemon=True)
    t1.start()
    t2.start()
    print("Threading System Running... Press Ctrl+C to stop.")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped.")