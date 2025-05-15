import paho.mqtt.client as mqtt
import time
import json # Might need this for sending/receiving structured data

# Configuration (maybe load from config/mqtt_config.json)
BROKER_ADDRESS = '127.0.0.1'
BROKER_PORT = 1883
CLIENT_ID = "rpi_delivery_robot"

# Global flag (managed by callbacks)
is_connected = False

# --- Callback Functions ---
def on_connect(client, userdata, flags, rc):
    global is_connected
    if rc == 0:
        is_connected = True
        print(f"Connected to MQTT Broker! Return code {rc}")
        # Subscribe to topics after successful connection
        subscribe_to_topics(client) 
    else:
        print(f"Failed to connect, return code {rc}\n")
        is_connected = False

def on_disconnect(client, userdata, rc):
    global is_connected
    print(f"Disconnected from MQTT Broker! Return code {rc}")
    is_connected = False # loop_start will try to reconnect

# Example sensor data callbacks (adapt/add as needed for your sensors)
def on_message_lidar(client, userdata, msg):
    # Process LiDAR data received from ESP32
    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    print(f"Received LiDAR data on {topic}: {payload}")
    # TODO: Parse payload (e.g., JSON string) and update robot state/map


def on_message_ultrasonic(client, userdata, msg):
    # Process Ultrasonic data received from ESP32
    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    print(f"Received Ultrasonic data on {topic}: {payload}")
    # TODO: Parse payload and use for obstacle avoidance


def on_message_imu(client, userdata, msg):
     # Process IMU data received from ESP32
    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    print(f"Received IMU data on {topic}: {payload}")
    # TODO: Parse payload and use for localization/orientation tracking

# Add more callbacks for other sensors or topics you subscribe to...

# --- Subscription Management ---
def subscribe_to_topics(client):
    # List all topics you want to subscribe to
    client.subscribe("/robot/data/sensors/lidar")
    client.subscribe("/robot/data/sensors/ultrasonic")
    client.subscribe("/robot/data/sensors/imu")
    # Add other sensor topics, or a wildcard like client.subscribe("/robot/data/sensors/#")
    # Add any other topics the RPi needs to listen on (e.g., robot status requests)

# --- Publishing Function ---
def publish_command(topic, payload):
    """
    Function for other modules to call to publish a message.
    Payload can be a string, bytes, or a dictionary/list that gets JSON encoded.
    """
    if not is_connected:
        print("Warning: MQTT client not connected, cannot publish.")
        return False
        
    if isinstance(payload, (dict, list)):
        payload = json.dumps(payload) # Convert dictionary/list to JSON string

    try:
        # qos=0 is fire-and-forget, qos=1 is at least once, qos=2 is exactly once
        # qos=1 or 2 adds overhead but guarantees delivery if connection is intermittent
        publish_result = client.publish(topic, payload.encode('utf-8'), qos=1) 
        # Optional: Wait for confirmation if guaranteed delivery is needed
        # publish_result.wait_for_publish() 
        # print(f"Message published to {topic}")
        return True
    except Exception as e:
        print(f"Failed to publish message to {topic}: {e}")
        return False

# --- MQTT Client Setup ---
client = mqtt.Client(CLIENT_ID) # Create the client instance

# Assign general callbacks
client.on_connect = on_connect
client.on_disconnect = on_disconnect

# Assign specific callbacks for subscribed topics
client.message_callback_add("/robot/data/sensors/lidar", on_message_lidar)
client.message_callback_add("/robot/data/sensors/ultrasonic", on_message_ultrasonic)
client.message_callback_add("/robot/data/sensors/imu", on_message_imu)
# Add .message_callback_add for any other specific topics you subscribe to

# --- Initialization and Loop ---
def start_mqtt_client():
    """
    Starts the MQTT client connection and background loop.
    Call this from your main run_robot.py script.
    """
    try:
        client.connect(BROKER_ADDRESS, BROKER_PORT)
        client.loop_start() # Start the thread
        print(f"Attempting to connect to MQTT broker at {BROKER_ADDRESS}:{BROKER_PORT}...")
    except Exception as e:
        print(f"Error connecting to MQTT broker: {e}")

def stop_mqtt_client():
     """
     Stops the MQTT client. Call this when shutting down the robot.
     """
     print("Stopping MQTT client...")
     client.loop_stop() # Stop the thread
     client.disconnect() # Disconnect from the broker

# Example usage (in your main run_robot.py or for testing)
if __name__ == "__main__":
    start_mqtt_client()

    # Keep the main script running (it will receive messages in the background thread)
    try:
        while True:
            time.sleep(1) 
            # Your main robot logic loop would be here, processing received data
            # and calling publish_command() to send motor commands
            # Example: check some condition and send a command
            # if some_condition:
            #     publish_command("/robot/cmd_vel", {"linear_vel": 0.2, "angular_vel": 0.0})

            # Simple status check (loop_start handles reconnection, but helps see status)
            if not is_connected:
                 print("MQTT Client is currently disconnected...")

    except KeyboardInterrupt:
        print("Shutting down robot...")
        stop_mqtt_client()
        print("Robot shut down complete.")