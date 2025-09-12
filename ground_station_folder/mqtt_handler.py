import paho.mqtt.client as mqtt
import json
from datetime import datetime


class MQTTHandler:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # Connection details
        self.broker = "broker.emqx.io"
        self.port = 1883
        
        # Topics
        self.topic_from_pi = "raspberry/pi_to_laptop"
        self.topic_to_pi = "raspberry/laptop_to_pi"
        self.status_topic = "raspberry/status"
        self.heartbeat_topic = "raspberry/heartbeat"
        
        # Error codes
        self.TEMP_ERROR = 12345678
        self.IMU_ERROR = 23456789
        self.GPS_ERROR = 34567890
        
        # Current readings storage (per drone)
        self.last_temp_reading = {}
        self.last_imu_reading = {}
        self.last_gps_reading = {}
        self.last_heartbeat = {}

        # Historical data
        self.sensor_data = []
        
        # Callbacks
        self.gui_update_callback = None
        self.map_update_callback = None
    
    def connect(self):
        """Connect to the MQTT broker"""
        self.client.will_set(self.status_topic, "offline", retain=True)
        self.client.connect(self.broker, self.port)
        self.client.loop_start()
    
    def on_connect(self, client, userdata, flags, rc):
        """Connection callback"""
        if rc == 0:
            client.subscribe(self.topic_from_pi)
            client.subscribe(self.status_topic)
            client.subscribe(self.heartbeat_topic)
            client.publish(self.status_topic, "online", retain=True)
    
    def on_message(self, client, userdata, msg):
        """Incoming message handler"""
        try:
            data = json.loads(msg.payload.decode())
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            if msg.topic == self.status_topic:
                return

            # MAVLink-style heartbeat messages
            if msg.topic == self.heartbeat_topic:
                system_id = data.get("system_id")
                if system_id is not None:
                    self.last_heartbeat[system_id] = timestamp
                return

            drone_id = data.get("drone_id", "unknown")
            sensor_type = data.get("sensor")
            value = data.get("data")

            # Store raw data
            processed_data = {
                'timestamp': timestamp,
                'drone_id': drone_id,
                'sensor': sensor_type,
                'value': value
            }
            self.sensor_data.append(processed_data)

            # Update current readings
            if sensor_type == "temperature":
                self.last_temp_reading[drone_id] = value
            elif sensor_type == "imu":
                self.last_imu_reading[drone_id] = value
            elif sensor_type == "gps":
                # Normalize GPS data
                if isinstance(value, list) and len(value) >= 2:
                    self.last_gps_reading[drone_id] = {"lat": value[0], "lon": value[1]}
                elif isinstance(value, dict):
                    self.last_gps_reading[drone_id] = value
                else:
                    self.last_gps_reading[drone_id] = {"lat": self.GPS_ERROR, "lon": self.GPS_ERROR}

            # Notify GUI
            if self.gui_update_callback:
                self.gui_update_callback(processed_data)

            # Update map if valid GPS
            if (
                sensor_type == "gps"
                and isinstance(self.last_gps_reading.get(drone_id), dict)
                and self.last_gps_reading[drone_id].get("lat") != self.GPS_ERROR
            ):
                if self.map_update_callback:
                    self.map_update_callback(
                        drone_id,
                        self.last_gps_reading[drone_id]["lat"],
                        self.last_gps_reading[drone_id]["lon"],
                    )

        except Exception as e:
            print(f"Error processing message: {e}")

    def send_command(self, command, action, target="all"):
        """Send commands to Raspberry Pi or a specific drone."""
        payload = {
            "command": command,
            "action": action,
            "target": target,
        }
        self.client.publish(self.topic_to_pi, json.dumps(payload))

    def send_heartbeat(
        self,
        system_id,
        component_id,
        autopilot=0,
        base_mode=0,
        custom_mode=0,
        system_status=0,
        mavlink_version=3,
    ):
        """Publish a MAVLink-style heartbeat message."""
        payload = {
            "type": "heartbeat",
            "system_id": system_id,
            "component_id": component_id,
            "autopilot": autopilot,
            "base_mode": base_mode,
            "custom_mode": custom_mode,
            "system_status": system_status,
            "mavlink_version": mavlink_version,
            "timestamp": datetime.utcnow().isoformat(),
        }
        self.client.publish(self.heartbeat_topic, json.dumps(payload))
    
    def get_sensor_data(self):
        """Get all collected sensor data"""
        return self.sensor_data
    
    def get_last_temp_reading(self, drone_id=None):
        """Get last temperature reading for a drone or return error code."""
        if drone_id is None:
            return next(iter(self.last_temp_reading.values()), self.TEMP_ERROR)
        return self.last_temp_reading.get(drone_id, self.TEMP_ERROR)

    def get_last_imu_reading(self, drone_id=None):
        """Get last IMU reading for a drone or return error code."""
        if drone_id is None:
            return next(iter(self.last_imu_reading.values()), self.IMU_ERROR)
        return self.last_imu_reading.get(drone_id, self.IMU_ERROR)

    def get_last_gps_reading(self, drone_id=None):
        """Get last GPS reading for a drone or return error dict."""
        if drone_id is None:
            gps = next(iter(self.last_gps_reading.values()), None)
        else:
            gps = self.last_gps_reading.get(drone_id)
        if not isinstance(gps, dict):
            return {"lat": self.GPS_ERROR, "lon": self.GPS_ERROR}
        return gps

    def temp_error_state(self, drone_id=None):
        """Check if temperature is in error state."""
        if drone_id is None:
            return any(v == self.TEMP_ERROR for v in self.last_temp_reading.values())
        return self.last_temp_reading.get(drone_id, self.TEMP_ERROR) == self.TEMP_ERROR

    def imu_error_state(self, drone_id=None):
        """Check if IMU is in error state."""
        if drone_id is None:
            return any(v == self.IMU_ERROR for v in self.last_imu_reading.values())
        return self.last_imu_reading.get(drone_id, self.IMU_ERROR) == self.IMU_ERROR

    def gps_error_state(self, drone_id=None):
        """Check if GPS is in error state."""
        if drone_id is None:
            return any(
                gps.get("lat") == self.GPS_ERROR or gps.get("lon") == self.GPS_ERROR
                for gps in self.last_gps_reading.values()
            )
        gps = self.last_gps_reading.get(drone_id, {})
        return gps.get("lat") == self.GPS_ERROR or gps.get("lon") == self.GPS_ERROR