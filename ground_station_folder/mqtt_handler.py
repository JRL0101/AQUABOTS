import paho.mqtt.client as mqtt
import json
import os
import time
from datetime import datetime

class MQTTHandler:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        # Load configuration
        config_path = os.path.join(os.path.dirname(__file__), '..', 'mqtt_config.json')
        with open(config_path) as f:
            config = json.load(f)

        # Connection details
        self.broker = config.get("broker", "localhost")
        self.port = config.get("port", 1883)

        # Topics
        topics = config.get("topics", {})
        self.topic_from_pi = topics.get("pi_to_laptop", "pi/to/laptop")
        self.topic_to_pi = topics.get("laptop_to_pi", "laptop/to/pi")
        self.status_topic = topics.get("status", "status")
        
        # Error codes
        self.TEMP_ERROR = 12345678
        self.IMU_ERROR = 23456789
        self.GPS_ERROR = 34567890
        
        # Current readings storage
        self.last_temp_reading = None
        self.last_imu_reading = None
        self.last_gps_reading = None
        
        # Historical data
        self.sensor_data = []
        
        # Callbacks
        self.gui_update_callback = None
        self.map_update_callback = None
    
    def connect(self):
        """Connect to the MQTT broker with automatic reconnect"""
        self.client.will_set(self.status_topic, "offline", retain=True)

        delay = 1
        while True:
            try:
                self.client.connect(self.broker, self.port)
                self.client.loop_start()
                break
            except Exception as e:
                print(f"MQTT connection failed: {e}. Retrying in {delay}s")
                time.sleep(delay)
                delay = min(delay * 2, 60)

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            delay = 1
            while True:
                try:
                    time.sleep(delay)
                    client.reconnect()
                    break
                except Exception as e:
                    print(f"MQTT reconnect failed: {e}. Retrying in {delay}s")
                    delay = min(delay * 2, 60)
    
    def on_connect(self, client, userdata, flags, rc):
        """Connection callback"""
        if rc == 0:
            client.subscribe(self.topic_from_pi)
            client.subscribe(self.status_topic)
            client.publish(self.status_topic, "online", retain=True)
    
    def on_message(self, client, userdata, msg):
        """Incoming message handler"""
        try:
            data = json.loads(msg.payload.decode())
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            if msg.topic == self.status_topic:
                return
            
            sensor_type = data.get("sensor")
            value = data.get("data")
            
            # Store raw data
            processed_data = {
                'timestamp': timestamp,
                'sensor': sensor_type,
                'value': value
            }
            self.sensor_data.append(processed_data)
            
            # Update current readings
            if sensor_type == "temperature":
                self.last_temp_reading = value
            elif sensor_type == "imu":
                self.last_imu_reading = value
            elif sensor_type == "gps":
                # Normalize GPS data
                if isinstance(value, list) and len(value) >= 2:
                    self.last_gps_reading = {"lat": value[0], "lon": value[1]}
                elif isinstance(value, dict):
                    self.last_gps_reading = value
                else:
                    self.last_gps_reading = {"lat": self.GPS_ERROR, "lon": self.GPS_ERROR}
            
            # Notify GUI
            if self.gui_update_callback:
                self.gui_update_callback(processed_data)
            
            # Update map if valid GPS
            if (sensor_type == "gps" and 
                isinstance(self.last_gps_reading, dict) and self.last_gps_reading.get("lat") != self.GPS_ERROR):
                if self.map_update_callback:
                    self.map_update_callback(
                        self.last_gps_reading["lat"],
                        self.last_gps_reading["lon"]
                    )
        
        except Exception as e:
            print(f"Error processing message: {e}")
    
    def send_command(self, command, action):
        """Send commands to Raspberry Pi"""
        payload = {
            "command": command,
            "action": action
        }
        self.client.publish(self.topic_to_pi, json.dumps(payload))
    
    def get_sensor_data(self):
        """Get all collected sensor data"""
        return self.sensor_data
    
    def get_last_temp_reading(self):
        """Get last temperature reading or return error code"""
        return getattr(self, 'last_temp_reading', self.TEMP_ERROR)

    def get_last_imu_reading(self):
        """Get last IMU reading or return error code"""
        return getattr(self, 'last_imu_reading', self.IMU_ERROR)

    def get_last_gps_reading(self):
        """Get last GPS reading or return error dict"""
        gps = getattr(self, 'last_gps_reading', None)
        if not isinstance(gps, dict):
            return {"lat": self.GPS_ERROR, "lon": self.GPS_ERROR}
        return gps
    
    def temp_error_state(self):
        """Check if temperature is in error state"""
        return self.last_temp_reading == self.TEMP_ERROR
    
    def imu_error_state(self):
        """Check if IMU is in error state"""
        return self.last_imu_reading == self.IMU_ERROR
    
    def gps_error_state(self):
        """Check if GPS is in error state"""
        return (isinstance(self.last_gps_reading, dict) and 
                (self.last_gps_reading.get("lat") == self.GPS_ERROR or self.last_gps_reading.get("lon") == self.GPS_ERROR))