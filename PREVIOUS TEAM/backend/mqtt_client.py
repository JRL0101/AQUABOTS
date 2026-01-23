try:
    import paho.mqtt.client as mqtt
except Exception:  # pragma: no cover - library not available in test env
    mqtt = None


class MQTTClient:
    """Simple wrapper around paho-mqtt with graceful fallbacks."""

    def __init__(self, broker: str, port: int, topic_sub: str, topic_pub: str) -> None:
        self.broker = broker
        self.port = port
        self.topic_sub = topic_sub
        self.topic_pub = topic_pub
        self.on_message = lambda message: None
        if mqtt is not None:
            self.client = mqtt.Client()
            self.client.on_connect = self._on_connect
            self.client.on_message = self._internal_on_message
        else:  # pragma: no cover - when mqtt library missing
            self.client = None

    def _on_connect(self, client, userdata, flags, rc):
        if self.client is not None:
            self.client.subscribe(self.topic_sub)

    def _internal_on_message(self, client, userdata, msg):
        self.on_message(msg.payload.decode())

    def set_message_callback(self, callback):
        self.on_message = callback

    def publish(self, message: str) -> None:
        if self.client is not None:
            try:
                self.client.publish(self.topic_pub, message)
            except Exception:
                pass

    def start(self) -> None:
        if self.client is not None:
            try:
                self.client.connect(self.broker, self.port, 60)
                self.client.loop_start()
            except Exception:
                pass

    def stop(self) -> None:
        if self.client is not None:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except Exception:
                pass
