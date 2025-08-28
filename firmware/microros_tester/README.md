# ESP32 ROS Communication Debug Firmware

This is a minimal debugging firmware that connects to WiFi and displays ROS message logs on a webpage. It helps isolate communication issues between the ROS2 stack and the ESP32.

## Features

- **WiFi Connection**: Connects to your local WiFi network
- **Web Interface**: Simple webpage showing real-time logs
- **ROS Message Logging**: Logs all received base and arm command messages
- **Joint State Publishing**: Sends back simulated joint states
- **Real-time Statistics**: Shows message counts and connection status

## Setup Instructions

### 1. Update WiFi Credentials
Edit `src/main.cpp` and change these lines:
```cpp
const char* ssid = "your_wifi_ssid";        // Replace with your WiFi network name
const char* password = "your_wifi_password"; // Replace with your WiFi password
```

### 2. Update ROS2 Machine IP
Find this line in `src/main.cpp` and change to your computer's IP:
```cpp
IPAddress agent_ip(192, 168, 1, 100);  // CHANGE THIS TO YOUR ROS2 MACHINE IP
```

### 3. Start micro-ROS Agent
On your ROS2 machine, run:
```bash
# Install micro-ROS agent if not already installed
sudo apt install ros-humble-micro-ros-agent

# Start the agent (WiFi mode)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 4. Upload Firmware
```bash
# From the firmware directory
pio run --target upload
```

### 5. Monitor
```bash
# Check serial output
pio device monitor

# Open web interface
# The ESP32 will print its IP address to serial
# Open http://ESP32_IP_ADDRESS in your browser
```

## Usage

1. **Launch your ROS2 bringup**: `ros2 launch leremix_bringup bringup.launch.py`
2. **Open the web interface** in your browser
3. **Send test commands**:
   ```bash
   # Test base command
   ros2 topic pub /esp32/base_cmd std_msgs/msg/Float64MultiArray "data: [0.1, -0.1, 0.0]"
   
   # Test arm command  
   ros2 topic pub /esp32/arm_cmd std_msgs/msg/Float64MultiArray "data: [0.0, 0.1, -0.1, 0.0, 0.0, 0.0, 0.0]"
   ```

## Web Interface

The webpage shows:
- **Statistics**: Uptime, message counts, WiFi signal strength
- **Real-time logs**: All received ROS messages with timestamps
- **Auto-refresh**: Updates every 2 seconds
- **Clear logs**: Visit `/clear` to reset counters

## Troubleshooting

1. **"micro-ROS initialization failed"**:
   - Check micro-ROS agent is running
   - Verify IP address is correct
   - Ensure firewall allows port 8888

2. **No messages received**:
   - Check ROS2 topics: `ros2 topic list`
   - Verify topic names match
   - Test with manual publish commands

3. **WiFi connection issues**:
   - Check SSID and password
   - Verify 2.4GHz network (ESP32 doesn't support 5GHz)
   - Check serial monitor for connection status

## Expected Output

When working correctly, you should see:
- Messages logged in real-time on the webpage
- Joint states published back to ROS2
- Message counters incrementing
- No error messages in logs

This will help isolate if the issue is:
- ✅ ROS2 → ESP32 communication (messages received)  
- ✅ ESP32 → ROS2 communication (joint states published)
- ❌ Servo hardware/control (isolated from communication)