import paho.mqtt.client as mqtt
import subprocess
import time
import os
import json
from threading import Thread, Event

# MQTT Configuration
BROKER = "broker.emqx.io"
PORT = 1883
TOPIC_TO_LAPTOP = "raspberry/pi_to_laptop"
TOPIC_FROM_LAPTOP = "raspberry/laptop_to_pi"
HEARTBEAT_TOPIC = "raspberry/heartbeat"

# Drone identifier
DRONE_ID = os.getenv("DRONE_ID", "drone_1")

# Paths to compiled C++ programs
TEMP_PROGRAM = "./temperature"
IMU_PROGRAM = "./imu_test_6050"
GPS_PROGRAM = "./GPS_test"
MotorOn = "./Bmotor_testOn"
MotorOff = "./Bmotor_testOff"

# Error codes
TEMP_ERROR = 12345678
IMU_ERROR = 23456789
GPS_ERROR = 34567890

class SensorHub:
    def __init__(self):
        # Use Event for thread control
        self.stop_event = Event()
        
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.connect(BROKER, PORT, 60)
        self.mqtt_client.loop_start()

        # Start sensor and heartbeat threads
        Thread(target=self.read_temperature, daemon=True).start()
        Thread(target=self.read_imu, daemon=True).start()
        Thread(target=self.read_gps, daemon=True).start()
        Thread(target=self.send_heartbeat, daemon=True).start()
        
    def on_mqtt_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(TOPIC_FROM_LAPTOP)
        
    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            if msg.topic == TOPIC_FROM_LAPTOP:
                command = payload.get("command")
                action = payload.get("action")
                target = payload.get("target", "all")

                if target in ("all", DRONE_ID):
                    if command == "emergency" and action == "shutdown":
                        subprocess.run(["sudo", MotorOff])
                        print("EMERGENCY STOP RECEIVED - SHUTTING DOWN")
                        self.stop()
                    elif command == "control":
                        if action == "start":
                            subprocess.run(["sudo", MotorOn])
                        elif action == "stop":
                            subprocess.run(["sudo", MotorOff])

        except Exception as e:
            print(f"Error processing command: {str(e)}")

    def send_heartbeat(self):
        while not self.stop_event.is_set():
            payload = {
                "type": "heartbeat",
                "system_id": DRONE_ID,
                "component_id": 1,
                "autopilot": 0,
                "base_mode": 0,
                "custom_mode": 0,
                "system_status": 0,
                "mavlink_version": 3,
                "timestamp": time.time(),
            }
            self.mqtt_client.publish(HEARTBEAT_TOPIC, json.dumps(payload))
            time.sleep(1)
    
    def read_temperature(self):
        while not self.stop_event.is_set():
            try:
                result = subprocess.run([TEMP_PROGRAM], capture_output=True, text=True, timeout=5)
                temp = None
                
                for line in result.stdout.split('\n'):
                    if "Temperature:" in line:
                        temp = float(line.split()[1].replace('°C', ''))
                        break
                
                self.send_data("temperature", temp if temp is not None else TEMP_ERROR)
                
            except Exception as e:
                print(f"Temperature error: {str(e)}")
                self.send_data("temperature", TEMP_ERROR)
            
            time.sleep(5)
    
    def read_imu(self):
        while not self.stop_event.is_set():
            try:
                # Run with timeout shorter than your GUI update interval
                result = subprocess.run([IMU_PROGRAM], capture_output=True, text=True, timeout=1.0)
            
                # Process all output lines
                for line in result.stdout.splitlines():
                    line = line.strip()
                    if line.startswith('{') and line.endswith('}'):
                        try:
                            data = json.loads(line)
                        
                            # More careful error checking
                            if not any(v == 23456789 or abs(v) > 1000 
                                    for v in [data.get('accel_x'), 
                                          data.get('accel_y'),
                                          data.get('accel_z'),
                                          data.get('gyro_x'),
                                          data.get('gyro_y'),
                                          data.get('gyro_z')]):
                            
                                # Only send if all values look reasonable
                                self.send_data("imu", data)
                                break
                            
                        except (json.JSONDecodeError, TypeError):
                            continue
            
                # If no valid data found in output
                else:
                    self.send_data("imu", {
                        "accel_x": 23456789,
                        "accel_y": 23456789,
                        "accel_z": 23456789,
                        "gyro_x": 23456789,
                        "gyro_y": 23456789,
                        "gyro_z": 23456789
                    })
                
            except subprocess.TimeoutExpired:
                self.send_data("imu", {
                    "accel_x": 23456789,
                    "accel_y": 23456789,
                    "accel_z": 23456789,
                    "gyro_x": 23456789,
                    "gyro_y": 23456789,
                    "gyro_z": 23456789
                })
            
            except Exception as e:
                print(f"IMU communication error: {str(e)}")
                self.send_data("imu", {
                    "accel_x": 23456789,
                    "accel_y": 23456789,
                    "accel_z": 23456789,
                    "gyro_x": 23456789,
                    "gyro_y": 23456789,
                    "gyro_z": 23456789
                })
        
            time.sleep(0.1)  # Match this with your GUI update rate

    
    def read_gps(self):
        while not self.stop_event.is_set():
            try:
                result = subprocess.run([GPS_PROGRAM], capture_output=True, text=True, timeout=10)
                gps_data = None
            
                # Check stdout for JSON data
                for line in result.stdout.split('\n'):
                    if line.startswith('{') and line.endswith('}'):
                        try:
                            data = json.loads(line)
                            if all(key in data for key in ['lat', 'lon', 'altitude']):
                                gps_data = {
                                    "lat": data['lat'],
                                    "lon": data['lon'],
                                    "altitude": data['altitude']
                                }
                                break
                        except json.JSONDecodeError:
                            continue
            
                self.send_data("gps", gps_data if gps_data else GPS_ERROR)
            
            except subprocess.TimeoutExpired:
                # Program is running continuously, timeout is expected
                continue
            except Exception as e:
                print(f"GPS error: {str(e)}")
                self.send_data("gps", GPS_ERROR)
        
            time.sleep(1)

    
    def send_data(self, sensor_type, data):
        payload = {
            "drone_id": DRONE_ID,
            "sensor": sensor_type,
            "data": data,
            "timestamp": time.time()
        }
        self.mqtt_client.publish(TOPIC_TO_LAPTOP, json.dumps(payload))
    
    def stop(self):
        """Proper shutdown sequence"""
        print("Initiating shutdown...")
        
        # Signal threads to stop
        self.stop_event.set()
        
        # Clean up MQTT
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        
        # Force exit
        os._exit(0)

if __name__ == "__main__":
    # Compile programs if needed
    if not os.path.exists(IMU_PROGRAM):
        print("Compiling IMU code...")
        os.system("g++ imu_test_6050.cpp -o imu_test_6050 -lwiringPi")
    
    if not os.path.exists(GPS_PROGRAM):
        print("Compiling GPS code...")
        os.system("g++ GPS_test.cpp -o GPS_test -lwiringPi")
    
    if not os.path.exists(TEMP_PROGRAM):
        print("Compiling temperature code...")
        os.system("g++ temperature.cpp -o temperature")
    
    # Verify programs exist
    required_programs = {
        "IMU": IMU_PROGRAM,
        "GPS": GPS_PROGRAM,
        "Temperature": TEMP_PROGRAM
    }
    
    missing = [name for name, path in required_programs.items() if not os.path.exists(path)]
    if missing:
        print(f"Error: Missing compiled programs for {', '.join(missing)}")
        exit(1)
    
    # Set executable permissions
    for program in required_programs.values():
        os.chmod(program, 0o755)
    
    # Run sensor hub
    try:
        hub = SensorHub()
        print("Sensor hub running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        hub.stop()







