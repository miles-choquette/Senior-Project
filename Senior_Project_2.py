import asyncio
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons
from bleak import BleakScanner
from collections import deque
from scipy.optimize import minimize
from filterpy.kalman import KalmanFilter
from PIL import Image
import requests
import pandas as pd
import boto3


# === CONFIGURATION ===
node_size_m = 0.05
img = Image.open("Downloads/NewScan.pgm").convert('L')
img_array = np.array(img)
img_height, img_width = img_array.shape
room_width_m = node_size_m * img_width
room_height_m = node_size_m * img_height
aws_url = "https://zhfu60mdh7.execute-api.us-east-2.amazonaws.com/Test/path"

# Beacon positions
beacons = {
    "A404ABC4-3F21-5B71-51D4-EA0AA6EC10D9": (0.15, 0.15),
    "702E6879-1856-EBB8-AA48-4F9B571D39FE": (0.15, 4.08),
    "4CCAF6FF-3A30-9D14-ADAE-B08A6B7892E7": (12.65, 4.08),
    "F0DF6AB5-4DBE-1D28-D347-1B0DEEE23002": (12.65, 0.15),
    "7FE6E2E2-18E1-B648-7457-BAA0D16658FE": (10.67, 0.91),
    "1FF363BC-25F6-A3A4-18DC-E41BA83C628C": (2.13, 2.13)
}
beacon_labels = {mac: f"Beacon {i+1}" for i, mac in enumerate(beacons)}

# Kalman Filter
kf = KalmanFilter(dim_x=2, dim_z=2)
kf.F = np.eye(2)
kf.H = np.eye(2)
kf.P *= 500
kf.R *= 2

P_TX = -59
PATH_LOSS_EXPONENT = 2.5
RSSI_HISTORY_SIZE = 5
rssi_history = {mac: deque(maxlen=RSSI_HISTORY_SIZE) for mac in beacons}

user_positions = []
returned_path = []
user_mobility_profile = ["default"]
clicks = []

# Load nodes
nodes_df = pd.read_csv("Downloads/Nodes (2).csv")

map_origin_x = -9.34
map_origin_y = -10.5

# map_origin_x = -5.49
# map_origin_y = -6.05

# --- Helper functions ---
def rssi_to_distance(rssi):
    return 10 ** ((P_TX - rssi) / (10 * PATH_LOSS_EXPONENT))

def smoothed_rssi(device):
    rssi_history[device.address].append(device.rssi)
    return sum(rssi_history[device.address]) / len(rssi_history[device.address])

def multilateration(distances):
    if len(distances) < 3:
        return None
    positions = [np.array(beacons[mac]) for mac in distances]
    dists = [distances[mac] for mac in distances]
    def error(p):
        return sum((np.linalg.norm(p - pos) - d) ** 2 for pos, d in zip(positions, dists))
    guess = np.mean(positions, axis=0)
    result = minimize(error, guess, method='L-BFGS-B')
    return result.x if result.success else None

def kalman_update(pos):
    kf.predict()
    kf.update(pos)
    return kf.x.copy()

def node_id_to_coords(nid):
    node = nodes_df.iloc[nid]
    return node['x_coordinate'], node['y_coordinate']

def coords_to_node_id(x, y):
    distances = np.sqrt((nodes_df['x_coordinate'] - x)**2 + (nodes_df['y_coordinate'] - y)**2)
    closest_idx = distances.idxmin()
    if distances[closest_idx] > 0.75:
        print("❌ No nearby node within 0.75 meters.")
        return None
    print(f"Clicked (world): ({x:.2f}, {y:.2f}) → Closest node ID: {closest_idx}")
    print(f"Node {closest_idx} coordinates: ({nodes_df.iloc[closest_idx]['x_coordinate']:.2f}, {nodes_df.iloc[closest_idx]['y_coordinate']:.2f})")
    return closest_idx

def world_to_image_coords(x, y):
    return ((x - map_origin_x) / room_width_m * img_width, img_height - ((y - map_origin_y) / room_height_m * img_height))

def distance_to_path(px, py):
    return min(np.linalg.norm(np.array([px, py]) - np.array(node_id_to_coords(n))) for n in returned_path) if returned_path else float('inf')

def draw_path():
    if not returned_path:
        return
    coords = [node_id_to_coords(n) for n in returned_path]
    pixels = [world_to_image_coords(x, y) for x, y in coords]
    x, y = zip(*pixels)
    ax.plot(x, y, 'b-', linewidth=2, label="Path")
    ax.plot(x[0], y[0], 'go', markersize=10, label="Start")
    ax.plot(x[-1], y[-1], 'ro', markersize=10, label="Goal")
    ax.legend()
    fig.canvas.draw()

def send_request():
    if len(clicks) < 2:
        print("Please select start and destination nodes.")
        return
    payload = {
        "start_id": clicks[0],
        "goal_id": clicks[1],
        "interest_nodes": [],
        "footprint_type": user_mobility_profile[0]
    }
    print(f"Sending to AWS: {payload}")
    try:
        response = requests.post(aws_url, json=payload)
        if response.status_code == 200:
            result = response.json()
            path = result.get("path", [])
            returned_path.clear()
            ax.clear()
            ax.imshow(img_array, cmap='gray', origin='upper')
            if not path:
                ax.text(0.5, 0.95, '❌ No path returned by AWS.', transform=ax.transAxes,
                        fontsize=14, color='red', ha='center', bbox=dict(facecolor='white', edgecolor='red'))
                print("❌ No path returned.")
            else:
                print(f"✅ Path received from AWS ({len(path)} nodes): {path}")
                returned_path.extend(path)
                draw_path()
            fig.canvas.draw()
        else:
            print("AWS error", response.status_code)
    except Exception as e:
        print("AWS request failed", e)

def on_submit(event):
    send_request()

def on_reset(event):
    clicks.clear()
    returned_path.clear()
    ax.clear()
    ax.imshow(img_array, cmap='gray', origin='upper')
    fig.canvas.draw()

def on_radio_clicked(label):
    user_mobility_profile[0] = label
    print(f"Profile: {label}")

def onclick(event):
    if event.xdata and event.ydata:
        img_px = event.xdata
        img_py = event.ydata

        x_m = img_px * (room_width_m / img_width) + map_origin_x
        y_m = (img_height - img_py) * (room_height_m / img_height) + map_origin_y

        nid = coords_to_node_id(x_m, y_m)
        if nid is None:
            return

        matched_x, matched_y = node_id_to_coords(nid)
        px, py = world_to_image_coords(matched_x, matched_y)
        ax.plot(px, py, 'yo', markersize=6)

        if len(clicks) >= 2:
            print("⚠️ You've already selected both start and destination. Please press Reset before selecting again.")
            return

        clicks.append(nid)
        color = 'go' if len(clicks) == 1 else 'ro'
        ax.plot(event.xdata, event.ydata, color, markersize=10)
        fig.canvas.draw()
        print(f"Node {len(clicks)} selected: ID = {nid}")

# --- Plot Setup ---
plt.ioff()
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(left=0.3)
ax.imshow(img_array, cmap='gray', origin='upper')
ax.axis('off')

# Radio Buttons
ax_radio = plt.axes([0.05, 0.4, 0.15, 0.2])
radio = RadioButtons(ax_radio, ('default', 'wheelchair', 'crutches'))
radio.on_clicked(on_radio_clicked)

# Buttons
submit_ax = plt.axes([0.05, 0.25, 0.1, 0.05])
submit_button = Button(submit_ax, 'Submit')
submit_button.on_clicked(on_submit)

reset_ax = plt.axes([0.05, 0.18, 0.1, 0.05])
reset_button = Button(reset_ax, 'Reset')
reset_button.on_clicked(on_reset)

# Click event for start and goal
fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()
