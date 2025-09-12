import paho.mqtt.client as mqtt
import time
import json
import os

# Load MQTT settings from config
CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'mqtt_config.json')
with open(CONFIG_PATH) as f:
    _config = json.load(f)

broker = _config.get("broker", "localhost")
port = _config.get("port", 1883)

# Topics - now matching both directions
topic_from_laptop = _config.get("topics", {}).get("laptop_to_pi", "laptop/to/pi")
topic_to_laptop = _config.get("topics", {}).get("pi_to_laptop", "pi/to/laptop")

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    # Subscribe to the laptop's topic
    client.subscribe(topic_from_laptop)

def on_message(client, userdata, msg):
    print(f"Received from Laptop: {msg.payload.decode()}")
    # Respond back to laptop
    client.publish(topic_to_laptop, payload="Pi received: " + msg.payload.decode(), qos=1)

# Set up MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

def connect_with_backoff():
    delay = 1
    while True:
        try:
            client.connect(broker, port, 60)
            break
        except Exception as e:
            print(f"MQTT connection failed: {e}. Retrying in {delay}s")
            time.sleep(delay)
            delay = min(delay * 2, 60)

def on_disconnect(client, userdata, rc):
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

client.on_disconnect = on_disconnect

# Connect to the broker with backoff
connect_with_backoff()

# Start the loop to listen for messages
client.loop_start()

try:
    while True:
        # You can add periodic messages here if needed
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
    client.loop_stop()
