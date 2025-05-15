# raspberry_pi/communication/mqtt_client_pi.py
import paho.mqtt.client as mqtt
import time
import json 
import sys 
# Import the specific CallbackAPIVersion enum
from paho.mqtt.client import CallbackAPIVersion 

# ... later in your code where you create the client ...

# --- Configuration ---
# Set the IP address of your MQTT broker. 
# If running Mosquitto on the SAME Raspberry Pi, use '127.0.0.1'.
# If running Mosquitto on a DIFFERENT machine (like your laptop for testing), use its IP address.
BROKER_ADDRESS = '127.0.0.1' # the IP address of my laptop 
BROKER_PORT = 1883 # Default MQTT port

# Client ID for this Raspberry Pi instance. Must be unique on the network.
CLIENT_ID = "rpi_delivery_robot" 

# --- Global State ---
# Use a global variable to track connection status, updated by callbacks.
is_mqtt_connected = False 

# Create the MQTT client instance (global so callback functions can access it)

client = mqtt.Client(client_id=CLIENT_ID, callback_api_version=CallbackAPIVersion.VERSION1)
# --- Callback Functions ---

def on_connect(client, userdata, flags, rc):
    """Callback function for when the client connects to the broker."""
    global is_mqtt_connected
    if rc == 0:
        is_mqtt_connected = True
        print(f"MQTT Client Connected to Broker! Return code: {rc}")
        # Subscribe to topics AFTER a successful connection.
        # This handles re-subscriptions if the client disconnects and reconnects.
        subscribe_to_topics(client) 
    else:
        is_mqtt_connected = False
        print(f"MQTT Client Failed to connect, return code: {rc}\n")
        # Depending on the error code (rc), you might add specific handling here.

def on_disconnect(client, userdata, rc):
    """Callback function for when the client disconnects from the broker."""
    global is_mqtt_connected
    is_mqtt_connected = False
    if rc != 0:
        print(f"MQTT Client Unexpectedly disconnected. Return code: {rc}")
    else:
        print(f"MQTT Client Disconnected cleanly. Return code: {rc}")

# --- Sensor Data Callbacks (from ESP32) ---
# These functions are triggered when a message arrives on a subscribed sensor topic.

def on_message_imu(client, userdata, msg):
    """Handles messages received on the esp32/IMU topic."""
    topic = msg.topic
    try:
        # Attempt to decode and potentially parse the payload (e.g., if it's JSON)
        payload_string = msg.payload.decode('utf-8')
        print(f"Received data on topic '{topic}': {payload_string}")
        # TODO: Add logic here to parse payload_string (e.g., using json.loads())
        # and update your robot's internal state (e.g., orientation variable)
        pass # Placeholder - remove when you add actual processing
    except Exception as e:
        print(f"Error processing message on topic '{topic}': {e}")

def on_message_gps(client, userdata, msg):
    """Handles messages received on the esp32/GPS topic."""
    topic = msg.topic
    try:
        payload_string = msg.payload.decode('utf-8')
        print(f"Received data on topic '{topic}': {payload_string}")
        # TODO: Add logic here to parse payload_string (e.g., lat/lon)
        # and update your robot's internal state (e.g., position variable)
        pass # Placeholder - remove when you add actual processing
    except Exception as e:
        print(f"Error processing message on topic '{topic}': {e}")

def on_message_lidar(client, userdata, msg):
    """Handles messages received on the esp32/LIDAR topic."""
    topic = msg.topic
    try:
        payload_string = msg.payload.decode('utf-8')
        print(f"Received data on topic '{topic}': {payload_string}")
        # TODO: Add logic here to parse payload_string (e.g., the comma-separated list or JSON array)
        # and update your robot's internal state (e.g., LiDAR scan data)
        pass # Placeholder - remove when you add actual processing
    except Exception as e:
        print(f"Error processing message on topic '{topic}': {e}")

# --- Example General Broadcast Callback ---
# Based on the topic in your previous code. You can keep, modify, or remove this.
def on_message_rpi_broadcast(client, userdata, msg):
    """Handles messages on the rpi/broadcast topic (e.g., for general communication)."""
    topic = msg.topic
    try:
        payload_string = msg.payload.decode('utf-8')
        print(f"Received broadcast on topic '{topic}': {payload_string}")
        # TODO: Add logic if this topic is used for specific tasks
        pass # Placeholder
    except Exception as e:
        print(f"Error processing message on topic '{topic}': {e}")


# --- Subscription Management ---

def subscribe_to_topics(client):
    """Subscribes the client to all necessary topics."""
    print("Subscribing to topics...")
    # Subscribe to the specific sensor topics from the ESP32 code
    client.subscribe("esp32/IMU")
    client.subscribe("esp32/GPS")
    client.subscribe("esp32/LIDAR")
    
    # You can also subscribe to a wildcard for flexibility (like esp32/#)
    # This catches anything published by ESP32 starting with esp32/
    # client.subscribe("esp32/#") 
    
    # Subscribe to the broadcast topic
    client.subscribe("rpi/broadcast")

    # Add subscriptions for any other topics this RPi needs to listen to.
    # E.g., for commands FROM a remote control panel: client.subscribe("control_panel/commands")

    print("Subscription complete.")

# --- Publishing Function ---

def publish_message(topic, payload, qos=0, retain=False):
    """
    Publishes a message to an MQTT topic.
    This is the function other parts of your RPi code will call to send commands/data.

    Args:
        topic (str): The topic to publish to (e.g., "/robot/cmd_vel").
        payload: The message payload. Can be string, bytes, int, float, dict, list.
                 Dictionaries/lists will be automatically JSON encoded.
        qos (int): Quality of Service (0, 1, or 2). Default 0 (fire and forget).
        retain (bool): Whether the message should be retained by the broker.
                       False for commands, maybe True for configuration/status.
    """
    if not is_mqtt_connected:
        # print(f"Warning: Not connected to MQTT broker, cannot publish to {topic}.")
        # It's often okay to fail silently here if the loop_start() handles reconnection
        return False

    # Automatically encode dicts/lists as JSON strings
    if isinstance(payload, (dict, list)):
        payload = json.dumps(payload)
    # Ensure payload is bytes
    if isinstance(payload, str):
        payload = payload.encode('utf-8')
    elif not isinstance(payload, bytes):
         # Handle other types if necessary, e.g., convert numbers to string/bytes
         payload = str(payload).encode('utf-8')


    try:
        # The publish call itself is usually non-blocking when using loop_start()
        # The delivery happens in the background thread.
        # result = client.publish(topic, payload, qos=qos, retain=retain)
        # You can get more info (like mid) if needed, but often not necessary for simple publishes.
        client.publish(topic, payload, qos=qos, retain=retain)
        # print(f"Published to {topic}: {payload.decode('utf-8')}") # Optional print
        return True
    except Exception as e:
        print(f"Error publishing to {topic}: {e}")
        return False

# --- MQTT Client Setup ---

def setup_mqtt_client():
    """Assigns callbacks and sets up the MQTT client properties."""
    # Assign the general callbacks
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect

    # Assign specific callbacks for subscribed topics
    # Add message_callback_add for each specific topic you want separate handling for
    client.message_callback_add("esp32/IMU", on_message_imu)
    client.message_callback_add("esp32/GPS", on_message_gps)
    client.message_callback_add("esp32/LIDAR", on_message_lidar)
    client.message_callback_add("rpi/broadcast", on_message_rpi_broadcast) # If you keep this topic

    # Note: If you subscribed to a wildcard like "esp32/#" using client.subscribe(),
    # any message arriving on a topic within that wildcard that *doesn't* have a 
    # specific callback assigned with message_callback_add() will be handled by 
    # the client.on_message callback IF you set one (which we haven't here).
    # It's generally cleaner to use message_callback_add for all topics you expect.


# --- Initialization and Main Loop Control ---

def start_mqtt_client():
    """
    Sets up the client, attempts connection, and starts the background loop.
    Call this once at the start of your main robot script.
    """
    print("Starting MQTT Client setup...")
    setup_mqtt_client() # Assign callbacks etc.

    try:
        # Attempt to connect to the broker
        client.connect(BROKER_ADDRESS, BROKER_PORT)
        # Start the background thread for network loop.
        # This allows the main part of your script to continue running.
        client.loop_start() 
        print(f"Attempting to connect to MQTT broker at {BROKER_ADDRESS}:{BROKER_PORT} in background...")
        # Connection status will be confirmed by on_connect callback later.
        
    except ConnectionRefusedError:
        print(f"Connection refused. Is the MQTT broker running at {BROKER_ADDRESS}:{BROKER_PORT}?")
        # Handle this fatal setup error
        sys.exit("MQTT Broker connection refused.")
    except Exception as e:
        print(f"An error occurred during MQTT client start: {e}")
        sys.exit("Failed to start MQTT client.")


def stop_mqtt_client():
    """
    Stops the MQTT client background loop and disconnects cleanly.
    Call this when your robot script is shutting down.
    """
    print("Stopping MQTT Client...")
    if client:
        client.loop_stop() # Stop the background thread
        client.disconnect() # Send disconnect message to broker
        print("MQTT Client loop stopped and disconnected.")
    else:
        print("MQTT Client was not initialized.")

# --- Example Usage (for testing this file directly) ---

if __name__ == "__main__":
    # This code runs only when you execute this file directly (python your_file_name.py)
    # It demonstrates how to start the client and publish/receive messages.

    start_mqtt_client() # Start the MQTT client

    # Keep the main script running in a loop.
    # In your actual robot script, this loop would be your main robot control logic.
    try:
        print("MQTT client is running in the background.")
        print("Keep this window open. Messages will print as they arrive.")
        print("Press Ctrl+C to exit.")

        # Demonstrate publishing a test message periodically
        publish_count = 0
        while True:
            # Check if connected before trying to publish (optional, publish_message handles it)
            if is_mqtt_connected:
                 test_topic = "rpi/test_message" # A new topic just for this test
                 test_payload = f"Test message from RPi {publish_count}"
                 publish_message(test_topic, test_payload)
                 publish_count += 1
                 # You could also publish commands here, e.g.:
                 # publish_message("/robot/cmd_vel", {"linear": 0.1, "angular": 0.0})
                 # publish_message("rpi/broadcast", "Still alive!")


            time.sleep(5) # Publish a test message every 5 seconds


    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down.")

    except Exception as e:
        print(f"An error occurred in the main test loop: {e}")

    finally:
        stop_mqtt_client()
        print("MQTT Client test finished.")