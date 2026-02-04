# UWB-Based Drone Positioning System

### Project Overview
This repository houses a robust, Python-based positioning framework designed to enable autonomous drone navigation in GPS-denied environments using Ultra-Wideband (UWB) technology. The system employs Least Squares Multilateration algorithms to calculate precise real-time coordinates (x, y, z) based on distance measurements from UWB anchors.

Key capabilities include a custom GPS Emulation Layer that transforms local metric coordinates into global WGS84 standards and injects them into Pixhawk flight controllers via the NMEA 0183 protocol (specifically GPRMC and GPGGA sentences). The project offers flexible architectural implementations ranging from a stable synchronous loop to advanced Asyncio and Threading models for high-concurrency performance. Additionally, it features MAVSDK and ZMQ integration, allowing for dynamic anchor positioning and data sharing within multi-agent drone swarms.

## 🚀 Key Features

* **Real-Time Trilateration:** Uses Least Squares optimization to solve position from distance measurements.
* **GPS Emulation:** Generates standard NMEA sentences (`GPRMC`, `GPGGA`) to "trick" the flight controller into thinking it has a GPS fix.
* **Coordinate Conversion:** Automatically converts local metric (x, y) coordinates to WGS84 (Lat/Lon) using UTM projection.
* **Outlier Filtering:** Includes buffer management and threshold-based filtering to ignore noisy UWB data.
* **Multiple Architectures:** Includes synchronous (stable), swarm-based (MAVSDK), and experimental (Asyncio/Threading) implementations.

## 📂 Repository Structure

The project is organized to separate stable release code from experimental methods:

| File Name | Description |
| :--- | :--- |
| **`main_standalone.py`** | **[RECOMMENDED]** The most stable, standalone version. Uses hardcoded anchor coordinates. Ideal for single-drone testing and ground tests. |
| **`main_swarm.py`** | Advanced version that integrates with **MAVSDK** and **ZMQ**. It can receive dynamic anchor positions from other agents/drones in a swarm. |
| **`listener.py`** | A helper module for `main_swarm.py` to handle ZMQ network communication. |
| **`experimental/`** | Contains research & development codes (Threading and Asyncio approaches) for architectural reference. |

## 🛠️ Hardware Requirements

* **Onboard Computer:** Raspberry Pi, Jetson Nano, or similar (Linux recommended).
* **UWB Module:** Any serial-based UWB tag (e.g., Decawave, Nooploop) outputting distance as CSV (`d1,d2,d3`).
* **Flight Controller:** Pixhawk (ArduPilot or PX4 firmware) with a spare Telemetry/Serial port.

## ⚙️ Installation

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/Mertcagliyan/uwb-drone-positioning.git](https://github.com/Mertcagliyan/uwb-drone-positioning.git)
    cd uwb-drone-positioning
    ```

2.  **Install dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

## 🏃 Usage

### 1. Standalone Mode (Testing)
This is the easiest way to test the algorithm.
1.  Open `main_standalone.py`.
2.  Update `ANCHOR_GPS_POINTS` with the real GPS coordinates of your UWB anchors.
3.  Check `UWB_PORT` (e.g., `/dev/ttyUSB0`) and `PIXHAWK_PORT`.
4.  Run:
    ```bash
    python main_standalone.py
    ```

### 2. Swarm Mode (Advanced)
If you are running a swarm or receiving anchor data via network:
```bash
python main_swarm.py
