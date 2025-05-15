// #include <ArduinoJson.h> // Needed if you want to parse JSON commands from RPi (recommended later)
#include <WiFi.h>          // For Wi-Fi connection
#include <PubSubClient.h>  // For MQTT client functionality

// --- Configuration ---
// Replace with your actual Wi-Fi credentials
const char* ssid = "SSID"; 
const char* password = "PASSWORD of the SSID";

// Replace with the IP address of your MQTT Broker (e.g., your Raspberry Pi's IP)
const char* mqtt_server = "The address of the device connected to a stable WIFI!!"; // my laptop ip address on the network 
const int mqtt_port = 1883; // Default MQTT port

// Unique Client ID for this ESP32
const char* mqtt_client_id = "esp32_delivery_robot"; 

// --- MQTT Topics ---
// Topics this ESP32 will SUBSCRIBE to (commands from RPi)
const char* command_topic = "/robot/cmd_vel"; // Example topic for velocity commands

// Topics this ESP32 will PUBLISH to (sensor data to RPi)
const char* imu_topic = "esp32/IMU";         // Matches RPi subscription
const char* gps_topic = "esp32/GPS";         // Matches RPi subscription
const char* lidar_topic = "esp32/LIDAR";       // Matches RPi subscription
const char* ultrasonic_topic = "esp32/ultrasonic1"; // Example for one ultrasonic sensor
// Add more topics for other sensors...

// --- Global Variables ---
WiFiClient espClient;         // TCP/IP client for Wi-Fi connection
PubSubClient client(espClient); // MQTT client using the Wi-Fi client

long lastSensorPublishTime = 0; // Timer to control periodic sensor publishing (for dummy data)
const long publishInterval = 1000; // Publish sensor data every 1000ms (1 second) - adjust as needed

// Placeholder variables for dummy sensor data
float dummy_imu_heading = 0.0;
String dummy_gps_coords = "{\"lat\": 0.0, \"lon\": 0.0}";
String dummy_lidar_data = "0,0,0,0,0,0,0,0,0,0"; // Comma-separated dummy data
int dummy_ultrasonic_distance = 50; // Dummy distance in cm

// Placeholder variables for motor commands received from RPi
float target_linear_velocity = 0.0; // m/s
float target_angular_velocity = 0.0; // rad/s or deg/s

#define ledPin 2 // Pin for onboard LED (useful for debugging status)

// --- Function Declarations ---
void setup_wifi();
void connect_mqttServer();
void callback(char* topic, byte* message, unsigned int length);
void publish_sensor_data(); // Function to collect and publish sensor data
void process_commands(byte* message, unsigned int length); // Function to handle received commands
void set_motor_velocities(float linear, float angular); // Stub/placeholder for motor control

// --- LED Blink Function (Useful for Status Indication) ---
void blink_led(unsigned int times, unsigned int duration){
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(duration);
    digitalWrite(ledPin, LOW); 
    if (i < times - 1) delay(200); // Delay between blinks, not after the last one
  }
}

// --- Wi-Fi Setup ---
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    blink_led(1, 100); // Blink LED once quickly while trying to connect
    delay(500);       // Wait 0.5 seconds
    Serial.print(".");
    attempts++;
    if(attempts > 20){ // Restart after about 10 seconds of trying
        Serial.println("\nWiFi connection failed, restarting...");
        ESP.restart();
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  blink_led(5, 50); // Blink LED 5 times quickly on successful connection
}

// --- MQTT Connect and Reconnect Logic ---
void connect_mqttServer() {
  // Loop until we're reconnected
  while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect with unique client ID
        if (client.connect(mqtt_client_id)) {
          Serial.println("connected");
          // --- Subscribe to topics here after successful connection ---
          // Subscribe to the command topic from the RPi
          client.subscribe(command_topic);
          Serial.print("Subscribed to topic: ");
          Serial.println(command_topic);

          // You might also subscribe to other control/status topics here
          // client.subscribe("rpi/status_requests"); 
          
          // Blink LED to indicate MQTT connected
          blink_led(2, 200); 
          
        } 
        else {
          Serial.print("failed, rc=");
          Serial.print(client.state()); // Print state code for debugging
          Serial.println(" trying again in 5 seconds");
          // Wait 5 seconds before retrying
          delay(5000);
        }
  }
}

// --- MQTT Message Callback ---
// This function is called by client.loop() when a message arrives on a subscribed topic
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  // Note: message is a byte array, length is its size. It's NOT null-terminated.

  // --- Process Commands from RPi ---
  if (String(topic) == command_topic) {
      Serial.println("Received command message.");
      process_commands(message, length); // Hand off to a dedicated function
  }

  // --- Add checks for other subscribed topics if needed ---
  // else if (String(topic) == "some/other/topic") {
  //    // Process messages from that topic
  // }

  // Optional: You can print the raw message bytes if needed for debugging
  // Serial.print("Message: ");
  // for (int i = 0; i < length; i++) {
  //   Serial.print((char)message[i]);
  // }
  // Serial.println();
}

// --- Command Processing Logic ---
// This function takes the raw message from the command_topic and extracts actions
void process_commands(byte* message, unsigned int length) {
    // Example: Assuming RPi sends simple string commands like "forward", "stop", "turn_left"
    // For more complex commands (like velocities), you'd parse JSON here.

    String command = "";
    for (int i = 0; i < length; i++) {
        command += (char)message[i];
    }
    Serial.print("Parsed command string: ");
    Serial.println(command);

    // --- Basic String Command Parsing Example ---
    if (command == "forward") {
        Serial.println("Command: Move Forward");
        set_motor_velocities(0.3, 0.0); // Example: set target speed/turn rate
    } else if (command == "stop") {
        Serial.println("Command: Stop");
        set_motor_velocities(0.0, 0.0);
    } else if (command == "turn_left") {
        Serial.println("Command: Turn Left");
        set_motor_velocities(0.0, 0.5); // Example: set turn rate
    } 
    // TODO: Add more command handling (e.g., for target velocities)
    // For target velocities, you'd likely receive JSON: {"linear": 0.3, "angular": 0.1}
    // You would use ArduinoJson library to parse this.
    /* Example JSON parsing (requires ArduinoJson library):
    StaticJsonDocument<64> doc; // Adjust size as needed
    DeserializationError error = deserializeJson(doc, message, length);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return; // Exit if parsing fails
    }
    float linear_vel = doc["linear"] | 0.0; // Get "linear", default to 0.0 if not found
    float angular_vel = doc["angular"] | 0.0; // Get "angular", default to 0.0 if not found
    Serial.print("Received V_linear: "); Serial.print(linear_vel);
    Serial.print(", V_angular: "); Serial.println(angular_vel);
    set_motor_velocities(linear_vel, angular_vel);
    */
}

// --- Sensor Data Collection and Publishing ---
void publish_sensor_data() {
  // This function should read your actual sensors or return dummy data,
  // format it (e.g., into JSON), and publish it.

  // --- Read Actual/Dummy Sensor Data ---
  // TODO: Replace dummy data assignment with calls to your sensor reading functions
  // dummy_imu_heading = readIMUSensor(); 
  // dummy_gps_coords = readGPSSensor(); // Format as JSON string
  // dummy_lidar_data = readLidarSensor(); // Format as string (e.g., comma-separated or JSON)
  // dummy_ultrasonic_distance = readUltrasonicSensor();

  // --- Publish Data to MQTT ---
  // Publish IMU data
  // Example payload could be just the heading or a JSON string: {"heading": 90.5}
  // For simple values like heading, just sending the number as a string is easiest initially.
  // client.publish(imu_topic, String(dummy_imu_heading).c_str()); // Publish heading as string

  // Publish GPS data (assuming it's already formatted as a JSON string)
  // client.publish(gps_topic, dummy_gps_coords.c_str()); 

  // Publish LiDAR data (assuming it's formatted as a string)
  // client.publish(lidar_topic, dummy_lidar_data.c_str());

  // Publish Ultrasonic data (example as simple value)
  // client.publish(ultrasonic_topic, String(dummy_ultrasonic_distance).c_str());

  // --- Example: Publishing dummy IMU data periodically (like the original code) ---
  // This is just to show publishing works. You should replace this
  // with actual sensor reading and publishing logic.
  dummy_imu_heading += 1.0; // Increment dummy heading for testing
  if (dummy_imu_heading >= 360.0) dummy_imu_heading = 0.0;
  
  // Format dummy data as JSON (requires ArduinoJson library)
  /*
  StaticJsonDocument<64> doc;
  doc["heading"] = dummy_imu_heading;
  char jsonBuffer[64]; // Adjust size as needed
  serializeJson(doc, jsonBuffer);
  client.publish(imu_topic, jsonBuffer); // Publish JSON string
  */
 
  // Simpler: Publish as plain string
  client.publish(imu_topic, String(dummy_imu_heading).c_str()); // Publish as plain string
  Serial.print("Published dummy IMU heading: ");
  Serial.println(dummy_imu_heading);


  // TODO: Add similar blocks to read and publish other sensor data
  // e.g., read ultrasonic, publish ultrasonic_topic
  // e.g., read lidar, publish lidar_topic
  // e.g., read gps, publish gps_topic
  
}

// --- Motor Control Stub/Placeholder ---
// This function takes desired speeds and applies them to the motors.
// Initially, it will just print what it's being asked to do.
void set_motor_velocities(float linear, float angular) {
  // TODO: Replace this with your actual motor control logic
  // This involves calculating individual wheel speeds from linear/angular velocities
  // and sending commands (PWM signals, direction pins) to your motor driver board.

  Serial.print("Motor Control Stub: Received V_linear = ");
  Serial.print(linear);
  Serial.print(", V_angular = ");
  Serial.println(angular);

  // Example: Simple differential drive kinematics placeholder
  // (You'll need your wheel radius and track width for real kinematics)
  float left_wheel_speed = linear - angular; // Simplified example
  float right_wheel_speed = linear + angular; // Simplified example

  // TODO: Convert left_wheel_speed and right_wheel_speed into motor PWM values/commands
  // and send to your motor driver.
  Serial.print("Motor Control Stub: Calculated Left Wheel Speed = ");
  Serial.print(left_wheel_speed);
  Serial.print(", Right Wheel Speed = ");
  Serial.println(right_wheel_speed);

  // Example: control actual motor driver (replace with your driver code)
  // motorDriver.setSpeed(LEFT_MOTOR, left_wheel_speed_converted_to_pwm);
  // motorDriver.setSpeed(RIGHT_MOTOR, right_wheel_speed_converted_to_pwm);
}


// --- Arduino Setup Function ---
void setup() {
  pinMode(ledPin, OUTPUT); // Configure the LED pin as an output
  Serial.begin(115200);    // Initialize primary serial for debugging output

  // Connect to Wi-Fi
  setup_wifi();

  // Configure MQTT client server and callback
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); // Assign the function to be called on incoming messages
  
  // Initial connection attempt is handled by the loop() function's check
  // connect_mqttServer(); // Not needed here, loop() does it

  // TODO: Initialize sensors here (e.g., begin IMU, GPS, configure ultrasonic pins)
  // imu.begin();
  // gps.begin();
  // pinMode(ultrasonic_trig_pin, OUTPUT);
  // pinMode(ultrasonic_echo_pin, INPUT);

  Serial.println("ESP32 Setup Complete.");
}

// --- Arduino Loop Function ---
void loop() {
  // Ensure the MQTT client stays connected. Reconnects if necessary.
  // This also handles calling client.loop() internally, which processes incoming messages
  // and calls the callback() function when messages are received.
  if (!client.connected()) {
    connect_mqttServer();
  }
  client.loop(); // MQTT client's background processing (important!)

  // --- Publish Sensor Data Periodically (for testing with dummy data) ---
  // Replace this with logic that publishes when new sensor data is actually available.
  long now = millis();
  if (now - lastSensorPublishTime > publishInterval) {
    lastSensorPublishTime = now;
    publish_sensor_data(); // Call the function to read (dummy) data and publish
  }

  // TODO: Add code here for continuous tasks that are NOT blocking
  // - Read actual sensors (non-blocking reads where possible)
  // - Update sensor data variables
  // - Run low-level motor control loops (e.g., PID if using encoders)
  // - Check for safety conditions (e.g., immediate stop if ultrasonic detects obstacle very close)
  // - Don't use delay() here unless it's very short, as it blocks MQTT and other tasks.
}

// --- TODO: Implement Actual Sensor Reading Functions ---
// float readIMUSensor() { ... return heading; }
// String readGPSSensor() { ... return jsonString; }
// String readLidarSensor() { ... return dataString; }
// int readUltrasonicSensor() { ... return distance_cm; }
