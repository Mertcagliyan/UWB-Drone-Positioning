import serial
import numpy as np
from scipy.optimize import least_squares
import time
from pyproj import Transformer
import pynmea2
from datetime import datetime, timezone

"""
Standalone UWB Positioning System (No MAVSDK required).

Description:
    This version performs UWB trilateration using FIXED (Hardcoded) Anchor coordinates.
    It is useful for testing the algorithm without needing external drone telemetry
    or a swarm network.
    
    Ideal for: Single drone testing, ground tests, or debugging UWB sensors.
"""

# --- Configuration Parameters ---
UWB_PORT = 'COM5'            # Port for UWB module
UWB_BAUD = 115200
PIXHAWK_PORT = 'COM3'        # Port for Flight Controller
PIXHAWK_BAUD = 38400

MAX_VALID_DISTANCE = 100     # Max valid UWB range (meters)
OUTLIER_THRESHOLD = 10       # Outlier filter threshold
DRONE_HEIGHT = 0.0           # Relative height difference

# --- MANUAL ANCHOR SETUP ---
# Since we don't use MAVSDK here, we manually define the GPS coordinates
# of the 3 anchors (UWB beacons).
# WARNING: Update these coordinates for your flight zone!
ANCHOR_GPS_POINTS = [
    (39.872126, 32.732462),  # Anchor 1 (Reference Origin)
    (39.872081, 32.732290),  # Anchor 2
    (39.872046, 32.732461)   # Anchor 3
]

previous_uwb_measurements = None

# --- Coordinate Transformations ---

def local_to_gps(x, y, reference_gps):
    """Converts Local Metric (x, y) -> Global GPS (Lat, Lon)."""
    # CRS: WGS84 (EPSG:4326) <-> UTM Zone 36N (EPSG:32636)
    transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)
    transformer_to_latlon = Transformer.from_crs("EPSG:32636", "EPSG:4326", always_xy=True)

    ref_lat, ref_lon = reference_gps
    x0, y0 = transformer_to_utm.transform(ref_lon, ref_lat)

    utm_x = x0 + x
    utm_y = y0 + y
    lon, lat = transformer_to_latlon.transform(utm_x, utm_y)
    return lat, lon

def gps_to_local(points):
    """Converts Anchor GPS points -> Local Metric Coordinates (x, y)."""
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)
    utm_points = [transformer.transform(lon, lat) for lat, lon in points]
    
    x0, y0 = utm_points[0] # First point is origin (0,0)
    local_coords = [(x - x0, y - y0) for x, y in utm_points]
    return local_coords

# Pre-calculate local coordinates of anchors
local_system = gps_to_local(ANCHOR_GPS_POINTS)
local_coordinates = [[x, y] for x, y in local_system]
reference_gps = [ANCHOR_GPS_POINTS[0][0], ANCHOR_GPS_POINTS[0][1]]

# --- Solver ---

def cost_function(p, uwb_measurements, anchor_coords):
    """Least Squares residual function."""
    residuals = []
    for d_meas, p_anchor in zip(uwb_measurements, anchor_coords):
        d_calc = np.linalg.norm(np.array(p) - p_anchor)
        residuals.append((d_calc - d_meas)**2)
    return residuals

def decimal_to_nmea(coord: float, is_lat: bool = True):
    """Formats decimal coordinate to NMEA string."""
    degrees = int(abs(coord))
    minutes = (abs(coord) - degrees) * 60
    deg_width = 2 if is_lat else 3
    deg_str = f"{degrees:0{deg_width}d}"
    min_str = f"{minutes:07.4f}"
    direction = ('N' if coord >= 0 else 'S') if is_lat else ('E' if coord >= 0 else 'W')
    return deg_str + min_str, direction

def read_serial_data(ser):
    """Reads UWB data from serial with outlier filtering."""
    global previous_uwb_measurements
    while True:
        try:
            # Buffer flush mechanism for low latency
            if ser.in_waiting > 100:
                ser.reset_input_buffer()
                continue

            data = ser.readline().decode('utf-8').strip()
            if not data:
                continue

            values = [float(val.strip()) for val in data.split(',')]
            if len(values) < 3:
                continue

            # Check validity
            if any(v <= 0 or v > MAX_VALID_DISTANCE for v in values):
                continue
            
            # Outlier check
            if previous_uwb_measurements is not None:
                deltas = [abs(a - b) for a, b in zip(values, previous_uwb_measurements)]
                if any(delta > OUTLIER_THRESHOLD for delta in deltas):
                    print(f"Outlier detected: {values}, skipping.")
                    continue
            
            previous_uwb_measurements = values.copy()
            yield values[:3]

        except ValueError:
            continue
        except Exception as e:
            print(f"Serial Read Error: {e}")
            continue

# --- Main Execution ---

if __name__ == "__main__":
    print("Starting Standalone UWB Positioning...")
    print(f"Anchors (Local): {local_coordinates}")

    try:
        # Initialize Serial Connections
        ser_uwb = serial.Serial(UWB_PORT, UWB_BAUD, timeout=0.1)
        # Using a context manager for Pixhawk is safer, but here we open globally
        # to match the logic of continuous writing.
        ser_pixhawk = serial.Serial(PIXHAWK_PORT, PIXHAWK_BAUD, timeout=0.1)
        
        print(f"Connected to UWB ({UWB_PORT}) and Pixhawk ({PIXHAWK_PORT})")
        ser_uwb.flushInput()

        for uwb_measurements in read_serial_data(ser_uwb):
            # 1. 3D -> 2D Projection
            uwb_measurements_2d = [np.sqrt(max(d**2 - DRONE_HEIGHT**2, 0)) for d in uwb_measurements]
            
            # 2. Trilateration
            initial_guess = [0, 0]
            result = least_squares(cost_function, initial_guess, args=(uwb_measurements_2d, local_coordinates))
            x, y = result.x
            
            # 3. Local -> GPS
            lat, lon = local_to_gps(x, y, reference_gps)
            
            print(f"Position: x={x:.2f}, y={y:.2f} -> GPS: {lat:.6f}, {lon:.6f}")
            print(f"UWB Distances: {uwb_measurements}")

            # 4. Generate NMEA
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
            
            # 5. Send to Pixhawk
            for s in (rmc, gga):
                line = str(s) + '\r\n'
                ser_pixhawk.write(line.encode('ascii'))
                # print(f"Sent NMEA: {line.strip()}")
            
            # Log to CSV (Optional)
            # timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            # with open("position_log.csv", "a") as f:
            #     f.write(f"{timestamp},{uwb_measurements},{x:.3f},{y:.3f},{lat:.6f},{lon:.6f}\n")

    except serial.SerialException as e:
        print(f"Serial Connection Failed: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if 'ser_uwb' in locals() and ser_uwb.is_open: ser_uwb.close()
        if 'ser_pixhawk' in locals() and ser_pixhawk.is_open: ser_pixhawk.close()