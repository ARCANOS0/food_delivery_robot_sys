import paho.mqtt.client as mqtt # Import the Paho MQTT client library for Python.
                                # This is the standard way to interact with MQTT brokers in Python.
                                # Alias it as 'mqtt' for easier use.

import time # Import the time library. Used here for pausing the script execution.

def on_connect(client, userdata, flags, rc):
   # Define a callback function named 'on_connect'. This function is called by the
   # MQTT client library whenever a successful connection to the broker is established.
   # Arguments:
   # - client: The client instance for this callback.
   # - userdata: Any user data set when creating the client.
   # - flags: Response flags from the broker.
   # - rc: The connection result code (0 means successful).

   global flag_connected # Declare that we intend to modify the global variable 'flag_connected'.
   flag_connected = 1  # Set the global flag to 1 to indicate that the client is connected.
   client_subscriptions(client) # Call the 'client_subscriptions' function to subscribe to topics.
                               # This is important because subscriptions need to be re-established
                               # if the client disconnects and reconnects.
   print("Connected to MQTT server") # Print a confirmation message to the console.

def on_disconnect(client, userdata, rc):
   # Define a callback function named 'on_disconnect'. This function is called by the
   # MQTT client library whenever the client disconnects from the broker.
   # Arguments: Same as on_connect, 'rc' indicates the reason for disconnection.

   global flag_connected # Declare intent to modify the global 'flag_connected'.
   flag_connected = 0  # Set the global flag to 0 to indicate that the client is disconnected.
   print("Disconnected from MQTT server") # Print a message to the console.
   
# a callback functions 
# The following functions are examples of callback functions that will be executed
# when a message is received on a specific, *subscribed* topic.

def callback_esp32_sensor1(client, userdata, msg):
    # Define a function to handle messages on the topic "esp32/sensor1".
    # Arguments are standard for message callbacks.
    print('ESP sensor1 data: ', msg.payload.decode('utf-8'))
    # Print a label and the message payload.
    # msg.payload is the message content as bytes. .decode('utf-8') converts it to a string.

def callback_esp32_sensor2(client, userdata, msg):
    # Define a function to handle messages on the topic "esp32/sensor2".
    # Similar to the sensor1 callback.
    print('ESP sensor2 data: ', str(msg.payload.decode('utf-8')))
    # str() around the decoded string is redundant here but harmless.

def callback_rpi_broadcast(client, userdata, msg):
    # Define a function to handle messages on the topic "rpi/broadcast".
    print('RPi Broadcast message:  ', str(msg.payload.decode('utf-8')))
    # This topic name suggests it might be used for messages *between* RPi components
    # or from RPi to other devices, but the callback is defined here on the RPi client,
    # so it would receive its own messages if it published to this topic and subscribed.
    # It's more typical for the ESP32 to subscribe to RPi topics for commands.

def client_subscriptions(client):
    # Define a helper function to manage the client's topic subscriptions.
    # It takes the MQTT client instance as an argument.
    client.subscribe("esp32/#") # Subscribe to all topics that start with "esp32/".
                               # The '#' is an MQTT wildcard meaning "any topic below this hierarchy".
                               # This is useful for receiving data from multiple sensors published
                               # by the ESP32 under topics like "esp32/sensor1", "esp32/lidar_data", etc.
    client.subscribe("rpi/broadcast") # Subscribe to the specific topic "rpi/broadcast".

client = mqtt.Client("rpi_client1") # Create an MQTT client instance.
                                   # "rpi_client1" is the unique client ID. If multiple
                                   # clients with the same ID connect, the broker will
                                   # typically disconnect the older one.

flag_connected = 0 # Initialize the global variable 'flag_connected' to 0 (disconnected).

# Assign the callback functions to the client instance's events:
client.on_connect = on_connect         # Call the 'on_connect' function when connected.
client.on_disconnect = on_disconnect   # Call the 'on_disconnect' function when disconnected.

# Assign specific callback functions to specific topics using message_callback_add:
# This is a more precise way than using the general client.on_message callback.
client.message_callback_add('esp32/sensor1', callback_esp32_sensor1) # When message arrives on 'esp32/sensor1', call callback_esp32_sensor1.
client.message_callback_add('esp32/sensor2', callback_esp32_sensor2) # When message arrives on 'esp32/sensor2', call callback_esp32_sensor2.
client.message_callback_add('rpi/broadcast', callback_rpi_broadcast) # When message arrives on 'rpi/broadcast', call callback_rpi_broadcast.

client.connect('127.0.0.1',1883) # Attempt to connect to the MQTT broker.
                                # '127.0.0.1' is the localhost IP address. This assumes
                                # the MQTT broker (like Mosquitto) is running directly
                                # on the same machine as this Python script (the Raspberry Pi).
                                # 1883 is the standard unencrypted MQTT port.

# start a new thread
client.loop_start() # Start a background thread that manages the network connection
                    # and handles sending/receiving MQTT messages. This allows the
                    # rest of your Python script to run without being blocked by
                    # waiting for MQTT traffic. It also automatically attempts
                    # to reconnect if the connection is lost.

client_subscriptions(client) # Call the subscription function. This is slightly redundant
                            # because on_connect also calls it, but ensures subscriptions
                            # are attempted immediately upon starting the loop thread.

print("......client setup complete............") # Print a message indicating the setup is finished.

# The following is the main part of your script that runs continuously
while True: # Start an infinite loop to keep the script running.
    time.sleep(4) # Pause the loop execution for 4 seconds.
    if (flag_connected != 1): # Check if the connection flag is not 1 (meaning disconnected).
        print("trying to connect MQTT server..") # If disconnected, print a message.
        # The paho client.loop_start() thread will automatically attempt reconnection,
        # so this message just indicates the status, it doesn't trigger reconnection itself.