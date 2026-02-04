import asyncio
import serial_asyncio
import numpy as np
from scipy.optimize import least_squares
from pyproj import Transformer
import pynmea2
from datetime import datetime, timezone

"""
Experimental Positioning System: Asyncio Approach

Description:
    This script utilizes Python's 'asyncio' library and 'pyserial-asyncio' 
    for true non-blocking I/O operations.
    
    Method:
    1. An async coroutine reads UWB data stream.
    2. Data is processed and pushed to an asyncio.Queue.
    3. Another coroutine consumes the queue and writes NMEA data to the Pixhawk.
    
    Requirements:
        pip install pyserial-asyncio
"""

# --- Configuration ---
UWB_PORT = 'COM5'
UWB_BAUD = 115200
PIXHAWK_PORT = 'COM3'
PIXHAWK_BAUD = 38400

MAX_VALID_DISTANCE = 100
OUTLIER_THRESHOLD = 5
DRONE_HEIGHT = 0.0

# Fixed Anchors
ANCHOR_GPS_POINTS = [
    (39.872126, 32.732462),
    (39.872081, 32.732290),
    (39.872046, 32.732461)
]

# Async Queue
data_queue = asyncio.Queue()
previous_uwb_measurements = None

# --- Helpers ---
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

# --- Async Coroutines ---

async def uwb_reader():
    """Async reader using serial_asyncio"""
    global previous_uwb_measurements
    try:
        reader, _ = await serial_asyncio.open_serial_connection(url=UWB_PORT, baudrate=UWB_BAUD)
        print(f"[Async UWB] Connected to {UWB_PORT}")
        
        while True:
            try:
                line = await reader.readline()
                data = line.decode(errors='ignore').strip()
                if not data: continue
                
                values = [float(val.strip()) for val in data.split(',') if val.strip()]
                if len(values) < 3: continue
                
                if any(v <= 0 or v > MAX_VALID_DISTANCE for v in values): continue

                if previous_uwb_measurements is not None:
                    deltas = [abs(a - b) for a, b in zip(values, previous_uwb_measurements)]
                    if any(delta > OUTLIER_THRESHOLD for delta in deltas):
                        continue
                previous_uwb_measurements = values.copy()

                # Process Data
                uwb_measurements_2d = [np.sqrt(max(d**2 - DRONE_HEIGHT**2, 0)) for d in values]
                result = least_squares(cost_function, [0, 0], args=(uwb_measurements_2d, local_coordinates))
                x, y = result.x
                lat, lon = local_to_gps(x, y, reference_gps)

                await data_queue.put((values, x, y, lat, lon))

            except Exception as e:
                print(f"[Reader Error] {e}")
                await asyncio.sleep(0.1)
                
    except Exception as e:
         print(f"[Async UWB] Connection Failed: {e}")

async def nmea_writer():
    """Async writer using serial_asyncio"""
    try:
        _, writer = await serial_asyncio.open_serial_connection(url=PIXHAWK_PORT, baudrate=PIXHAWK_BAUD)
        print(f"[Async Pixhawk] Connected to {PIXHAWK_PORT}")
        
        while True:
            try:
                uwb_values, x, y, lat, lon = await data_queue.get()
                
                dt = datetime.now(timezone.utc)
                time_str = dt.strftime("%H%M%S")
                date_str = dt.strftime("%d%m%y")
                
                lat_nmea, lat_dir = decimal_to_nmea(lat, True)
                lon_nmea, lon_dir = decimal_to_nmea(lon, False)

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
                    writer.write(line.encode('ascii'))

                await writer.drain()
                print(f"[Async NMEA] Sent Lat: {lat:.6f}, Lon: {lon:.6f}")
                
            except Exception as e:
                print(f"[Writer Error] {e}")
                await asyncio.sleep(0.1)

    except Exception as e:
         print(f"[Async Pixhawk] Connection Failed: {e}")

# --- Main Event Loop ---
async def main():
    task1 = asyncio.create_task(uwb_reader())
    task2 = asyncio.create_task(nmea_writer())
    await asyncio.gather(task1, task2)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopped by user.")