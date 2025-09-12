import paho.mqtt.client as mqtt
import subprocess
import time
import os
import json
from threading import Thread, Event

from w1thermsensor import W1ThermSensor, SensorNotReadyError
from smbus2 import SMBus
import serial
import pynmea2

# MQTT Configuration
BROKER = "broker.emqx.io"
PORT = 1883
TOPIC_TO_LAPTOP = "raspberry/pi_to_laptop"
TOPIC_FROM_LAPTOP = "raspberry/laptop_to_pi"

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

        # Initialize sensors
        try:
            self.temperature_sensor = W1ThermSensor()
        except Exception as e:
            print(f"Temperature sensor init error: {e}")
            self.temperature_sensor = None

        try:
            self.bus = SMBus(1)
            self.mpu_addr = 0x68
            # Wake MPU6050
            self.bus.write_byte_data(self.mpu_addr, 0x6B, 0)
        except Exception as e:
            print(f"IMU init error: {e}")
            self.bus = None
            self.mpu_addr = None

        try:
            self.gps_serial = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)
        except Exception as e:
            print(f"GPS init error: {e}")
            self.gps_serial = None

        # Start sensor threads
        Thread(target=self.read_temperature, daemon=True).start()
        Thread(target=self.read_imu, daemon=True).start()
        Thread(target=self.read_gps, daemon=True).start()
        
    def on_mqtt_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(TOPIC_FROM_LAPTOP)
        
    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            if msg.topic == TOPIC_FROM_LAPTOP:
                command = payload.get("command")
                action = payload.get("action")
                
                if command == "emergency" and action == "shutdown":
                    subprocess.run(["sudo","./Bmotor_testOff"])
                    print("EMERGENCY STOP RECEIVED - SHUTTING DOWN")
                    self.stop()
                    
        except Exception as e:
            print(f"Error processing command: {str(e)}")
    
    def read_temperature(self):
        while not self.stop_event.is_set():
            temp = TEMP_ERROR
            if self.temperature_sensor is not None:
                try:
                    temp = self.temperature_sensor.get_temperature()
                except SensorNotReadyError:
                    temp = TEMP_ERROR
                except Exception as e:
                    print(f"Temperature error: {str(e)}")
                    temp = TEMP_ERROR

            self.send_data("temperature", temp)
            time.sleep(5)
    
    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.mpu_addr, reg)
        low = self.bus.read_byte_data(self.mpu_addr, reg + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def read_imu(self):
        while not self.stop_event.is_set():
            if self.bus is not None:
                try:
                    accel_x = self._read_word(0x3B) / 16384.0
                    accel_y = self._read_word(0x3D) / 16384.0
                    accel_z = self._read_word(0x3F) / 16384.0
                    gyro_x = self._read_word(0x43) / 131.0
                    gyro_y = self._read_word(0x45) / 131.0
                    gyro_z = self._read_word(0x47) / 131.0
                    data = {
                        "accel_x": accel_x,
                        "accel_y": accel_y,
                        "accel_z": accel_z,
                        "gyro_x": gyro_x,
                        "gyro_y": gyro_y,
                        "gyro_z": gyro_z,
                    }
                    self.send_data("imu", data)
                except Exception as e:
                    print(f"IMU communication error: {str(e)}")
                    self.send_data("imu", {
                        "accel_x": IMU_ERROR,
                        "accel_y": IMU_ERROR,
                        "accel_z": IMU_ERROR,
                        "gyro_x": IMU_ERROR,
                        "gyro_y": IMU_ERROR,
                        "gyro_z": IMU_ERROR,
                    })
            else:
                self.send_data("imu", {
                    "accel_x": IMU_ERROR,
                    "accel_y": IMU_ERROR,
                    "accel_z": IMU_ERROR,
                    "gyro_x": IMU_ERROR,
                    "gyro_y": IMU_ERROR,
                    "gyro_z": IMU_ERROR,
                })

            time.sleep(0.1)

    
    def read_gps(self):
        while not self.stop_event.is_set():
            if self.gps_serial is not None:
                try:
                    line = self.gps_serial.readline().decode('ascii', errors='replace')
                    if line.startswith('$GPGGA'):
                        msg = pynmea2.parse(line)
                        gps_data = {
                            "lat": msg.latitude,
                            "lon": msg.longitude,
                            "altitude": getattr(msg, 'altitude', 0.0),
                        }
                        self.send_data("gps", gps_data)
                    else:
                        self.send_data("gps", GPS_ERROR)
                except Exception as e:
                    print(f"GPS error: {str(e)}")
                    self.send_data("gps", GPS_ERROR)
            else:
                self.send_data("gps", GPS_ERROR)

            time.sleep(1)

    
    def send_data(self, sensor_type, data):
        payload = {
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

        # Close hardware interfaces
        if hasattr(self, 'bus') and self.bus is not None:
            self.bus.close()
        if hasattr(self, 'gps_serial') and self.gps_serial is not None and self.gps_serial.is_open:
            self.gps_serial.close()

        # Force exit
        os._exit(0)

if __name__ == "__main__":
    try:
        hub = SensorHub()
        print("Sensor hub running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        hub.stop()







