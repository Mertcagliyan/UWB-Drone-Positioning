import serial
import numpy as np
from scipy.optimize import least_squares
import time
from pyproj import Transformer
import matplotlib.pyplot as plt
import pynmea2
import datetime
from mavsdk.telemetry import Position

# Custom imports
from listener import Listener

"""
UWB-Based Positioning System with MAVSDK Listener Integration.

Description:
    This script performs trilateration using UWB (Ultra-Wideband) distance measurements.
    It integrates with a 'Listener' class to fetch external drone positions (via ZMQ/MAVSDK)
    which can serve as dynamic anchors. The calculated position is converted to 
    GPS coordinates (WGS84) and sent to a Pixhawk flight controller via NMEA sentences.

    Coordinate System: UTM Zone 36N (Specific to Ankara/Turkey region).
"""

# --- Configuration Parameters ---
UWB_PORT = 'COM5'            # Serial port for UWB module
UWB_BAUD = 115200
PIXHAWK_PORT = 'COM3'        # Serial port for Pixhawk (Output)
PIXHAWK_BAUD = 38400

TIMEOUT = 0.1
OUTLIER_THRESHOLD = 5        # Max allowed jump between measurements (meters)
MAX_VALID_DISTANCE = 100     # Maximum reliable UWB range
DRONE_HEIGHT = 0.0           # Assumed relative altitude

previous_uwb_measurements = None
all_drone_pos: dict = {}

# --- Helper Functions ---

def position_decoder(msg: dict) -> Position:
    """Decodes a dictionary message into a MAVSDK Position object."""
    abs_alt = msg["absolute_altitude_m"] 
    lat = msg["latitude_deg"]
    lon = msg["longitude_deg"]
    rel_alt = msg["relative_altitude_m"]
    return Position(lat, lon, abs_alt, rel_alt)

async def extract_data(self, topic, msg: dict):
    """
    Callback function for the Listener class.
    Parses 'POSITION' topics and updates the global dictionary of drone positions.
    """
    if topic.decode() == "POSITION":
        drone_id = msg["drone_id"]
        pos = position_decoder(msg)
        self.other_drone_pos[drone_id] = pos

# Initialize Listener for external data (e.g., Swarm/Ground Station)
listen_ports = [102, 103, 104]
listener = Listener(listen_ports)
listener.append_awaitable(extract_data)


def local_to_gps(x, y, reference_gps):
    """
    Converts Local Metric (x, y) -> Global GPS (Lat, Lon).
    WARNING: CRS is set to EPSG:32636 (UTM Zone 36N - Ankara).
    Update 'EPSG:32636' if operating in a different UTM zone.
    """
    transformer_to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)
    transformer_to_latlon = Transformer.from_crs("EPSG:32636", "EPSG:4326", always_xy=True)

    ref_lat, ref_lon = reference_gps
    # Convert Reference GPS to UTM
    x0, y0 = transformer_to_utm.transform(ref_lon, ref_lat)

    # Apply local offset
    utm_x = x0 + x
    utm_y = y0 + y

    # Convert back to WGS84
    lon, lat = transformer_to_latlon.transform(utm_x, utm_y)
    return lat, lon

def gps_to_local(points):
    """
    Converts Global GPS points -> Local Metric (x, y) relative to the first point.
    WARNING: CRS is set to EPSG:32636 (UTM Zone 36N).
    """
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32636", always_xy=True)

    # Convert all points to UTM (Note: always_xy=True means input is lon, lat)
    utm_points = [transformer.transform(lon, lat) for lat, lon in points]

    # Set first point as origin (0,0)
    x0, y0 = utm_points[0]
    local_coords = [(x - x0, y - y0) for x, y in utm_points]

    return local_coords

def cost_function(p, uwb_measurements, anchor_coordinates):
    """
    Residual function for Least Squares optimization.
    Returns the difference between measured distances and calculated distances.
    """
    residuals = []
    for d_meas, p_anchor in zip(uwb_measurements, anchor_coordinates):
        d_calc = np.linalg.norm(np.array(p) - p_anchor)
        residuals.append((d_calc - d_meas)**2)
    return residuals

def decimal_to_nmea(coord: float, is_lat: bool = True):
    """Formats decimal coordinates into NMEA standard strings (ddmm.mmmm)."""
    degrees = int(abs(coord))
    minutes = (abs(coord) - degrees) * 60
    deg_width = 2 if is_lat else 3
    deg_str = f"{degrees:0{deg_width}d}"
    min_str = f"{minutes:07.4f}"
    
    if is_lat:
        direction = 'N' if coord >= 0 else 'S'
    else:
        direction = 'E' if coord >= 0 else 'W'
        
    return deg_str + min_str, direction

def read_serial_data(ser):
    """
    Generator to read and parse UWB distance data from serial port.
    Handles outlier detection and validation.
    """
    global previous_uwb_measurements
    while True:
        try:
            # Read line from serial
            data = ser.readline().decode('utf-8').strip()
            if not data:
                continue

            # Parse CSV
            values = [float(val.strip()) for val in data.split(',')]
            if len(values) < 3:
                continue

            # Validate range
            if any(v <= 0 or v > MAX_VALID_DISTANCE for v in values):
                print(f"Invalid value detected, skipping: {values}")
                continue
            
            # Outlier Filter
            if previous_uwb_measurements is not None:
                deltas = [abs(a - b) for a, b in zip(values, previous_uwb_measurements)]
                if any(delta > OUTLIER_THRESHOLD for delta in deltas):
                    print(f"Outlier detected (Delta: {deltas}), utilizing previous values.")
                    # Option: You can yield previous values or skip. Logic here skips to next read.
                    values = previous_uwb_measurements.copy() 
                    continue
            
            previous_uwb_measurements = values.copy()
            yield values[:3] 

        except ValueError:
            continue

# --- Main Execution ---

# Initialize output serial port
ser2 = serial.Serial(
    port=PIXHAWK_PORT,
    baudrate=PIXHAWK_BAUD,
    timeout=0.1,
    rtscts=False,
    dsrdtr=False,
    xonxoff=False
)

try:
    with serial.Serial(UWB_PORT, UWB_BAUD, TIMEOUT) as ser:
        ser.flushInput()
        print(f"Connected to UWB on {UWB_PORT} at {UWB_BAUD} baud.")
        
        for uwb_measurements in read_serial_data(ser):
            
            # Logic: Check if we have enough external drone data (Dynamic Anchors)
            # If not, the code falls back to manual coordinates defined below.
            if len(all_drone_pos) > 1:
                # Example of fetching dynamic anchor points from the Listener
                gps_points: list[float] = [
                    all_drone_pos["1"].latitude_deg, all_drone_pos["1"].longitude_deg,
                    all_drone_pos["2"].latitude_deg, all_drone_pos["2"].longitude_deg,
                    all_drone_pos["3"].latitude_deg, all_drone_pos["3"].longitude_deg
                ]
                
                local_system = gps_to_local(gps_points)
                print(f"Dynamic Anchors GPS: {gps_points}")

                # --- MANUAL OVERRIDE / FALLBACK ANCHORS ---
                # These were likely used for testing when dynamic data wasn't fully reliable.
                local_coordinates = [
                    [0.0, 0.0],  # Anchor 1 (Origin)
                    [0.0, 0.0],  # Anchor 2
                    [0.0, 0.0]   # Anchor 3
                ]
                reference_gps = [gps_points[0][0], gps_points[0][1]]

                # Assign calculated local coordinates to the solver input
                for i, (x, y) in enumerate(local_system):
                    local_coordinates[i][0] = x
                    local_coordinates[i][1] = y
                    print(f"Anchor {i} Local: {x:.2f}, {y:.2f}")

                # --- Calculation ---
                uwb_values_str = ', '.join(f"{val:.3f}" for val in uwb_measurements)
                print(f"UWB Measurements: {uwb_values_str}")
                
                # 3D to 2D Projection
                uwb_measurements_2d = [np.sqrt(max(d**2 - DRONE_HEIGHT**2, 0)) for d in uwb_measurements]
                
                # Solve Position
                initial_guess = [0, 0]
                result = least_squares(cost_function, initial_guess, args=(uwb_measurements_2d, local_coordinates))
                x, y = result.x
                print(f"Estimated Position (Local): x={x:.3f}, y={y:.3f}")
                
                lat, lon = local_to_gps(x, y, reference_gps)
                print(f"Estimated Position (GPS): lat={lat:.6f}, lon={lon:.6f}")

                # --- NMEA Transmission ---
                try:
                    dt = datetime.datetime.utcnow()
                    time_str = dt.strftime("%H%M%S")
                    date_str = dt.strftime("%d%m%y")

                    lat_nmea, lat_dir = decimal_to_nmea(lat, True)
                    lon_nmea, lon_dir = decimal_to_nmea(lon, False)

                    # RMC - Recommended Minimum
                    rmc = pynmea2.RMC('GP','RMC', (
                        time_str, 'A',
                        lat_nmea, lat_dir,
                        lon_nmea, lon_dir,
                        '0.0', '0.0',
                        date_str, '', '', 'A'
                    ))

                    # GGA - Fix Data
                    gga = pynmea2.GGA('GP','GGA', (
                        time_str,
                        lat_nmea, lat_dir,
                        lon_nmea, lon_dir,
                        '1', '10', '0.9',
                        f'{DRONE_HEIGHT:.1f}', 'M',
                        '0.0', 'M', '', ''
                    ))

                    # GSA - Satellite status
                    gsa = pynmea2.GSA('GP','GSA', (
                        'A', '3',
                        '01','02','03','04','05','06','07','08','','','','','',
                        '2.1','1.2','1.8'
                    ))

                    # VTG - Track made good
                    vtg = pynmea2.VTG('GP','VTG', ('0.0','T','','M','0.0','N','0.0','K','A'))

                    # Send to Serial
                    for s in (rmc, gga, gsa, vtg):
                        line = str(s) + '\r\n'
                        ser2.write(line.encode('ascii'))
                        print(f"Sent NMEA: {line.strip()}")
                        # Note: ser2.close() was in loop in original code, removed to keep connection open.
                        # If you need to close/reopen every time, uncomment ser2.close() 

                    time.sleep(0.2)  # 5 Hz refresh rate

                except KeyboardInterrupt:
                    ser.close()
            else:
                # Fallback or waiting state
                print("Waiting for drone position info (Swarm/Listener data)...")
                time.sleep(1)

except serial.SerialException:
    print(f"Failed to connect on {UWB_PORT}")
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    if ser2.is_open:
        ser2.close()