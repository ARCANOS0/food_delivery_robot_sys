import pygame
import math
import random # We'll use this to simulate random Lidar data

# import paho.mqtt.client as mqtt # Uncomment this if you add MQTT later

pygame.init()

# --- Constants ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60
LIDAR_RESOLUTION = 100


# Lidar/Visualization settings
# LIDAR_RESOLUTION is already defined from your start, but let's define max distance
# VISUALIZATION_RESOLUTION = LIDAR_RESOLUTION # Not directly used in this structure,
                                            # the data list length dictates the points drawn

MAX_LIDAR_DISTANCE = 500 # Assume Lidar max range is 500 units (e.g., cm, mm)
MAX_SCREEN_RADIUS = min(SCREEN_WIDTH, SCREEN_HEIGHT) // 2 - 20 # Max radius for drawing points on screen, minus some margin

# Scale factor to convert Lidar distance units to screen pixels
# Distance * scale = pixels
SCALE_FACTOR = MAX_SCREEN_RADIUS / MAX_LIDAR_DISTANCE

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GRAY = (100, 100, 100)

# --- Pygame Setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Lidar Visualization")
clock = pygame.time.Clock()

# --- Data Storage ---
# This list will hold the Lidar data as (angle, distance) tuples
# Angle in degrees (0-360), Distance in Lidar units (0 to MAX_LIDAR_DISTANCE)
lidar_data = []

# --- Coordinate Conversion Function ---
def polar_to_screen(angle_degrees, distance, center_x, center_y, scale):
    """
    Converts polar coordinates (angle, distance) to screen coordinates (x, y).
    Angle is in degrees (0-360). 0 degrees is typically right (positive x).
    Distance is in Lidar units.
    Center (center_x, center_y) is the Pygame screen position of the Lidar origin.
    Scale converts Lidar distance units to screen pixels.
    """
    # Convert angle to radians
    angle_rad = math.radians(angle_degrees)

    # Apply scale to distance
    scaled_distance = distance * scale

    # Calculate Cartesian coordinates relative to the origin (mathematical)
    # math.cos and math.sin expect radians
    # 0 degrees -> positive x, 90 degrees -> positive y (mathematical)
    relative_x = scaled_distance * math.cos(angle_rad)
    relative_y = scaled_distance * math.sin(angle_rad)

    # Convert to Pygame screen coordinates
    # Pygame origin is top-left (0,0)
    # X increases to the right (same as math)
    # Y increases downwards (opposite of math)
    screen_x = center_x + relative_x
    screen_y = center_y - relative_y # Subtract relative_y because Pygame Y is inverted

    # Return as integer coordinates for drawing
    return (int(screen_x), int(screen_y))

# --- Simulation Data Generation (Replace with MQTT later) ---
def simulate_lidar_data(num_points):
    """Generates a list of simulated lidar points (angle, distance)."""
    simulated_points = []
    angle_increment = 360.0 / num_points

    for i in range(num_points):
        angle = i * angle_increment
        # Simulate a distance: e.g., random or following a pattern
        # This simulates points roughly in a circle with some noise
        distance = random.uniform(MAX_LIDAR_DISTANCE * 0.1, MAX_LIDAR_DISTANCE * 0.8) # Random distance between 10%-80% max range

        simulated_points.append((angle, distance))
    return simulated_points

# --- MQTT Callbacks (Structure for later) ---
# def on_connect(client, userdata, flags, rc):
#     if rc == 0:
#         print("Connected to MQTT Broker!")
#         # Subscribe to your Lidar data topic here
#         # client.subscribe("your/lidar/topic")
#     else:
#         print("Failed to connect, return code %d\n", rc)

# def on_message(client, userdata, msg):
#     print(f"Received `{msg.payload.decode()}` from `{msg.topic}`")
#     # Parse the message payload to get angle and distance data
#     # Update the 'lidar_data' list (be mindful of thread safety if using loop_start)
#     # Example: Assuming payload is JSON [{"angle": 10, "distance": 200}, ...]
#     try:
#         new_scan_data = json.loads(msg.payload.decode())
#         # Convert new_scan_data into the (angle, distance) tuple format
#         # You'll need to handle how to replace or append to the lidar_data list
#         global lidar_data # Need to declare global if modifying the list directly
#         lidar_data = [(point['angle'], point['distance']) for point in new_scan_data]
#     except Exception as e:
#         print(f"Error processing MQTT message: {e}")


# --- MQTT Client Setup (Structure for later) ---
# client = mqtt.Client()
# client.on_connect = on_connect
# client.on_message = on_message
# client.connect("your_mqtt_broker_address", 1883, 60) # Replace with your broker details
# client.loop_start() # Starts a new thread to process network traffic

# --- Game Loop ---
running = True

while running:
    # --- Event Handling ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # --- Update Data ---
    # In the simulation: Generate a new scan each frame
    # In MQTT: The on_message callback would update the 'lidar_data' list
    lidar_data = simulate_lidar_data(LIDAR_RESOLUTION) # Replace with code to use MQTT data

    # --- Drawing ---
    screen.fill(BLACK) # Clear the screen with black background

    # Calculate the screen center (Lidar origin)
    center_x = SCREEN_WIDTH // 2
    center_y = SCREEN_HEIGHT // 2

    # Draw the Lidar origin (a small white circle)
    pygame.draw.circle(screen, WHITE, (center_x, center_y), 5)

    # Draw the Lidar points
    for angle, distance in lidar_data:
        # Convert Lidar (polar) data to screen (Cartesian) coordinates
        screen_pos = polar_to_screen(angle, distance, center_x, center_y, SCALE_FACTOR)

        # Ensure distance is within reasonable bounds before drawing
        if 0 <= distance <= MAX_LIDAR_DISTANCE:
            # Draw a small red circle at the calculated screen position
            # You could also draw a line from the center to the point
            pygame.draw.circle(screen, RED, screen_pos, 2)
            # pygame.draw.line(screen, GRAY, (center_x, center_y), screen_pos, 1) # Optional: draw beam lines

    # --- Update Display ---
    pygame.display.flip() # Update the full screen to show the drawing

    # --- Frame Rate Control ---
    clock.tick(FPS) # Limit the frame rate

# --- Clean up ---
# client.loop_stop() # Uncomment this if you started the MQTT loop thread
pygame.quit()