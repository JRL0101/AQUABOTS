import paho.mqtt.client as mqtt
import json
import subprocess
import threading
import os
from time import sleep

# Load MQTT configuration
CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'mqtt_config.json')
with open(CONFIG_PATH) as f:
    _config = json.load(f)

BROKER = _config.get("broker", "localhost")
PORT = _config.get("port", 1883)
TOPIC_FROM_LAPTOP = _config.get("topics", {}).get("laptop_to_pi", "laptop/to/pi")
TOPIC_TO_LAPTOP = _config.get("topics", {}).get("pi_to_laptop", "pi/to/laptop")

# Navigation program
NAVIGATION_PROGRAM = "./GNC_Code"
POST_NAVIGATION_SCRIPT = "./Bmotor_testOff"  # New script to run before state 0

class NavigationController:
    def __init__(self):
        self.state = 0
        self.pos_target = None
        self.nav_process = None
        self.stop_event = threading.Event()
        
        # Setup MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.connect_with_backoff()
        self.mqtt_client.loop_start()
        
        self.thread = threading.Thread(target=self.run_state_machine, daemon=True)
        self.thread.start()
    
    def run_external_script(self):
        """Execute the post-navigation script"""
        try:
            print("Running post-navigation script...")
            result = subprocess.run(["sudo","./Bmotor_testOff"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            if result.returncode != 0:
                print(f"Script failed with error: {result.stderr}")
            else:
                print(f"Script output: {result.stdout}")
                
        except Exception as e:
            print(f"Error running post-navigation script: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(TOPIC_FROM_LAPTOP)
    
    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            command = payload.get("command")
            action = payload.get("action")
            
            if command == "navigation" and self.state == 0:
                lat = action.get("lat")
                lon = action.get("lon")
                if lat is not None and lon is not None:
                    self.pos_target = (lat, lon)
                    print(f"Received navigation target: {self.pos_target}")
                    self.state = 1
                
            elif command == "control" and action == "stop":
                print("Received stop command")
                self.stop_navigation()
                
        except Exception as e:
            print(f"Error processing MQTT message: {e}")

    def connect_with_backoff(self):
        delay = 1
        while True:
            try:
                self.mqtt_client.connect(BROKER, PORT, 60)
                break
            except Exception as e:
                print(f"MQTT connection failed: {e}. Retrying in {delay}s")
                sleep(delay)
                delay = min(delay * 2, 60)

    def on_mqtt_disconnect(self, client, userdata, rc):
        if rc != 0:
            delay = 1
            while True:
                try:
                    sleep(delay)
                    client.reconnect()
                    break
                except Exception as e:
                    print(f"MQTT reconnect failed: {e}. Retrying in {delay}s")
                    delay = min(delay * 2, 60)
    
    def run_state_machine(self):
        while not self.stop_event.is_set():
            if self.state == 0:
                sleep(0.1)
                
            elif self.state == 1 and self.pos_target:
                lat, lon = self.pos_target
                print(f"Starting navigation to {lat}, {lon}")
                
                try:
                    self.nav_process = subprocess.Popen(
                        [NAVIGATION_PROGRAM, str(lat), str(lon)],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                    
                    while self.nav_process.poll() is None and self.state == 1:
                        sleep(0.1)
                        if self.stop_event.is_set():
                            self.nav_process.terminate()
                            break
                    
                    # Only run the post-script if navigation completed naturally (not stopped)
                    if self.state == 1:
                        print("Navigation completed normally")
                        self.run_external_script()  # Run the script before state transition
                    
                    # Reset state
                    self.state = 0
                    self.pos_target = None
                    self.send_navigation_complete()
                    
                except Exception as e:
                    print(f"Error during navigation: {e}")
                    self.state = 0
                    self.pos_target = None
    
    def stop_navigation(self):
        """Stop navigation and return to state 0 (without running post-script)"""
        if self.nav_process and self.nav_process.poll() is None:
            self.nav_process.terminate()
            print("Navigation stopped by user")
        
        self.state = 0
        self.pos_target = None
        self.send_navigation_complete()
    
    def send_navigation_complete(self):
        payload = {
            "status": "navigation_complete",
            "message": "Ready for new input"
        }
        self.mqtt_client.publish(TOPIC_TO_LAPTOP, json.dumps(payload))
    
    def shutdown(self):
        self.stop_event.set()
        self.stop_navigation()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
if __name__ == "__main__":
    controller = NavigationController()
    try:
        while True:
            sleep(1)
    except KeyboardInterrupt:
        controller.shutdown()









