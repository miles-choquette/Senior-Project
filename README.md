Indoor Positioning System - Bluetooth Beacon Triangulation
Real-time indoor localization system using Bluetooth Low Energy (BLE) beacons with Kalman filtering for assisted navigation in large indoor spaces.
Overview
This project was developed as part of a larger senior capstone focused on creating an accessible navigation solution for stadiums and large public venues. The system enables real-time user positioning to provide turn-by-turn directions optimized for accessibility needs (wheelchair, crutches, or default mobility profiles).
Problem Statement
Traditional GPS is unreliable indoors, making it difficult for people with disabilities to navigate large, complex spaces like stadiums, convention centers, and airports. Our solution provides accurate indoor positioning to support accessible route guidance and automatic path correction when users deviate from their route.
My Contribution
I developed the real-time user localization module that:

Continuously tracks user position using triangulation from 6 strategically placed BLE beacons
Detects when users deviate from their assigned route
Triggers path recalculation when off-track movement is detected
Provides position data to the navigation system for route correction
Implements Kalman filtering for smooth, noise-reduced position tracking

Technical Approach
Triangulation Algorithm:

Uses RSSI (Received Signal Strength Indicator) values from 6 BLE beacons
Converts RSSI to distance estimates using path loss model: distance = 10^((P_TX - RSSI) / (10 * n))

Reference transmit power (P_TX): -59 dBm
Path loss exponent (n): 2.5


Applies multilateration with scipy optimization (L-BFGS-B method) to calculate precise (x, y) coordinates
Implements 5-sample rolling average for RSSI smoothing to reduce signal noise
Uses Kalman filter for position state estimation and prediction

Deviation Detection:

Calculates real-time distance from user position to nearest path node
Triggers alerts when user strays beyond configurable threshold
Supports automatic path recalculation via AWS Lambda pathfinding API

Accessibility Profiles:

Three mobility profiles: default, wheelchair, crutches
Routes optimized based on selected profile constraints
Interactive map interface for path visualization

Technologies Used

Language: Python 3
Hardware: 6 Bluetooth Low Energy (BLE) beacons
Key Libraries:

bleak - Asynchronous BLE beacon scanning
scipy.optimize - Multilateration solver
filterpy - Kalman filtering implementation
numpy - Numerical computation
matplotlib - Real-time visualization and UI
pandas - Node graph data management


Cloud Integration: AWS Lambda (REST API) for pathfinding
Map Format: PGM (Portable Gray Map) with CSV node coordinates

System Architecture
[6 BLE Beacons] → [RSSI Collection & Smoothing] → [Distance Estimation]
                            ↓
                  [Multilateration Algorithm]
                            ↓
                    [Kalman Filter]
                            ↓
              [Position Output & Visualization]
                            ↓
              [Deviation Detection] → [Path Recalculation Trigger]
Installation
bash# Clone repository
git clone [your-repo-url]
cd [repo-name]

# Install dependencies
pip install bleak numpy scipy filterpy matplotlib Pillow requests pandas boto3

# Add required files to Downloads folder:
# - NewScan.pgm (floor plan map)
# - Nodes (2).csv (navigation graph nodes)
Beacon Configuration
Update beacon MAC addresses and positions in the script:
pythonbeacons = {
    "MAC_ADDRESS_1": (x1, y1),  # Position in meters
    "MAC_ADDRESS_2": (x2, y2),
    # ... 6 total beacons
}
Beacon positions should be measured in world coordinates (meters) relative to your map origin.
Usage
bashpython Senior_Project_2.py
Interactive Interface:

Select mobility profile (default/wheelchair/crutches) using radio buttons
Click on map to select start position (green marker)
Click again to select destination (red marker)
Press Submit to calculate and display accessible route
Press Reset to clear selections and start over

Live Tracking:

User position updates in real-time as BLE beacons are detected
System monitors distance to path and alerts on deviation
Automatic path recalculation can be triggered when off-route

Key Parameters

Positioning Accuracy: Dependent on beacon spacing and environment (~0.5-2m typical)
RSSI Smoothing: 5-sample rolling average
Path Loss Exponent: 2.5 (tuned for indoor environment)
Kalman Filter: 2D state space (x, y position)
Node Snap Threshold: 0.75m (maximum distance to match clicked position to node)
Map Resolution: 0.05m per pixel

System Components
Core Modules:

rssi_to_distance() - Path loss model for distance estimation
multilateration() - Scipy-based position solver
kalman_update() - State estimation and noise filtering
distance_to_path() - Deviation detection
world_to_image_coords() - Coordinate system transformation

AWS Integration:

REST API endpoint for accessibility-aware pathfinding
Payload includes start/goal nodes and mobility profile
Returns optimized node sequence for visualization

Future Improvements

Real-time BLE scanning integration (currently uses mock data pathway)
Dynamic beacon recalibration for changing environments
IMU sensor fusion for improved dead reckoning
Battery optimization for mobile deployment
Multi-floor support with elevation tracking
Historical path analytics and heatmap generation

Team Project Context
This positioning module was one component of a larger autonomous navigation robot system that:

Scanned and mapped indoor environments in real-time using LIDAR/camera
Generated PGM floor plans with accessibility node graphs
Calculated wheelchair-optimized routes avoiding stairs, narrow passages
Provided turn-by-turn navigation with real-time position correction
