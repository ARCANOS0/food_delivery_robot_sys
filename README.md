# هيكل المشروع : مهم 
my_delivery_robot/
├── README.md                 # Project overview, setup instructions, how to run
├── .gitignore                # Specify files to ignore in Git (build files, data, etc.)
├── LICENSE                   # Choose a license for your project (optional but good practice)
│
├── docs/                     # Project documentation, diagrams, reports
│   ├── architecture.md       # High-level system architecture diagram/description
│   ├── setup_guides/         # Hardware assembly, OS setup, library installation steps
│   ├── software_design.md    # Details of each module's responsibility and communication
│   └── ...                   # Other documentation
│
├── config/                   # Configuration files used by both RPi and ESP32 (e.g., MQTT topics, robot parameters)
│   ├── mqtt_config.json      # MQTT broker address, topics
│   ├── robot_params.yaml     # Wheel radius, track width, sensor offsets, etc.
│   └── ...
│
├── training/                 # Code and data for training the CV model (primarily on your PC)
│   ├── yolov5/               # Cloned/copied YOLOv5 repository or relevant training scripts
│   ├── datasets/             # Your custom dataset (images, labels)
│   │   ├── images/
│   │   │   ├── train/
│   │   │   ├── val/
│   │   │   └── test/
│   │   └── labels/
│   │       ├── train/
│   │       ├── val/
│   │       └── test/
│   ├── results/              # Output from training (trained weights, logs, etc.)
│   │   └── yolov5s_custom/   # Example for a specific training run
│   │       └── weights/
│   │           └── best.pt   # Your trained model!
│   └── train.py              # Your main training script (if you customize YOLOv5)
│
├── raspberry_pi/             # Code that runs on the Raspberry Pi 5
│   ├── requirements.txt      # Python dependencies for the RPi code (pip install -r requirements.txt)
│   ├── run_robot.py          # Main entry point for the RPi robot software
│   ├── main_controller/      # Orchestrates overall robot behavior, task management
│   │   ├── __init__.py
│   │   └── controller.py     # Main robot state machine, task execution
│   ├── navigation/           # Path planning and obstacle avoidance logic
│   │   ├── __init__.py
│   │   ├── navigator.py      # High-level navigation logic
│   │   ├── obstacle_avoider.py # Uses sensor/CV data for real-time avoidance
│   │   └── path_planner.py   # (Simple) path planning algorithm
│   ├── computer_vision/      # Camera interface and CV model inference
│   │   ├── __init__.py
│   │   ├── camera_feed.py    # Handles grabbing frames from RPi Cam Module 3
│   │   ├── object_detector.py # Loads model, runs inference, processes results
│   │   └── models/           # Location for *deployed* CV models (e.g., converted TFLite model)
│   │       └── yolov5s_custom.tflite # Or .pt, or .onnx depending on chosen method
│   ├── communication/        # Handles communication with ESP32 (MQTT client)
│   │   ├── __init__.py
│   │   └── mqtt_client_pi.py # Code to connect, subscribe, publish
│   ├── user_interface/       # Code for the screen UI or remote control interface
│   │   ├── __init__.py
│   │   └── display_ui.py     # (Optional) Code for the 7" screen display
│   └── utils/                # Helper functions
│       ├── __init__.py
│       └── robot_math.py     # Coordinate transformations, simple kinematics
│
└── esp32/                    # Code that runs on the ESP32
    ├── platformio.ini        # Configuration for PlatformIO (recommended framework)
    ├── src/                  # Source code files
    │   ├── main.cpp          # ESP32 main program (Arduino sketch or ESP-IDF entry point)
    │   ├── motor_control/
    │   │   ├── MotorDriver.h # Header for motor driver class
    │   │   └── MotorDriver.cpp # Implementation
    │   ├── sensors/
    │   │   ├── SensorManager.h # Manages multiple sensors
    │   │   ├── SensorManager.cpp
    │   │   ├── UltrasonicSensor.h # Example for one sensor type
    │   │   ├── UltrasonicSensor.cpp
    │   │   ├── LidarSensor.h
    │   │   ├── LidarSensor.cpp
    │   │   ├── ...           # Other sensor files
    │   ├── communication/
    │   │   ├── MqttClientEsp.h # Header for MQTT client
    │   │   └── MqttClientEsp.cpp # Implementation (uses PubSubClient or similar library)
    │   └── config.h          # ESP32 specific configuration (pins, WiFi creds, etc.)
    └── lib/                  # Libraries for ESP32 (e.g., PubSubClient, sensor libraries)
        └── Readme.md         # Explain how to add libraries (PlatformIO handles this)
