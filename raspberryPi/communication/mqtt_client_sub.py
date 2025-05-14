import paho.mqtt.client as mqtt # Import the Paho MQTT client library for Python.
import time # Import the time library. Used here for pausing the script execution.

from raspberryPi.communication.mqtt_client import client_subscriptions

# mqtt broker -- اعتبر ان البروكر ده عبارة عن سيرفر بيوزع وبيستقبل البيانات 
mqtt_broker = "mqtt.eclipse.org" # السيرفر بتاع صحاب المكتبة
local_mqtt_broker = "127.0.0.1" # سيرفر على الجهاز اللي حيشغل الكود ده
mqtt_port = 1883 

# raspberry pi 5 client
raspberry_pi_client = mqtt.Client("Raspberry pi 5")
raspberry_pi_client.connect(mqtt_broker) # connects the client to the borker "server"
flag_connected = 1 # عشان تعرف امتى الraspberrry pi متوصلة 


def on_connect(client, userdata, flag, rc) -> None: # rc is the connection result, 0 means successfully connected
    client_subscriptions(raspberry_pi_client)
    print("raspberry pi connected to the MQTT server")

def on_disconnect(client, userdata, rc) :
    flag_connected = 0 # means connection is lost -- الاشارة راحت يا مجدي
    print("Disconnected from the MQTT server")

# functions to receive the data from the esp sensors

# receives the data from the IMU sensor and prints the result
def esp32_IMU_recieve(client, userdata, msg) -> None: # msg == message -- بترجع البيانات اللي السينسور حيبعتها
    print("ESP32 IMU sensors received data is : ", msg.payload.decode("utf-8"))

# receives the data from the GPS sensor
def esp32_GPS_receive(client, userdata, msg) -> None:
    print("ESP32 GPS sensor received data is : ", msg.payload.decode("utf-8"))

# receives the data from the Lidar
def esp32_LIDAR_receive(client, userdata, msg) -> None:
    print("ESP32 LiDAR received data is : ", msg.payload.decode("utf-8"))


# خنخلي الراسبيري باي تتوصل بكل البيانات اللي بتتبعت من السينسورات عن طريق اننا بنوصلها بall topics
def client_sub_to_all(client) -> None:
    client.subscribe("esp32/#") #subscribe to all topic the begins with esp32

raspberry_pi_client.on_connect = on_connect
raspberry_pi_client.on_disconnect = on_disconnect

# subs to the topics that we need then print the received results by passing the functions
raspberry_pi_client.message_callback_add("esp32/IMU", esp32_IMU_recieve)
raspberry_pi_client.message_callback_add("esp32/GPS", esp32_GPS_receive)
raspberry_pi_client.message_callback_add("esp32/LIDAR", esp32_LIDAR_receive)
# connect the rapberry pi to the broker
raspberry_pi_client.connect(local_mqtt_broker, 1883)

# نبدأ نستقبل البيانات عن طريق الloop start
raspberry_pi_client.loop_start()
print(" --------------- Client Connected ---------------")

time.sleep(1)  # تأخير ثانية بين كل رسالة والتانية
raspberry_pi_client.loop_forever()