import time
import socket
import queue
import random
from threading import Thread
import paho.mqtt.client as mqtt

from typing import TYPE_CHECKING

from communication_format import *

if TYPE_CHECKING:
    from ground_station import GroundStation


class GroundStationServer:
    def __init__(self, _gs, ip, port):
        self.gs: GroundStation = _gs
        self.session_id = random.getrandbits(32)  # Generate a random 4-byte number
        self.drone_addresses = {drone["IP"] for drone in self.gs.file_manager.config["drones"]}
        queues = {address: queue.Queue() for address in self.gs.drones.keys()}
        self.drone_message_queues = queues
        self.pair_id = 0

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((ip, port))
        self.server_socket.listen()
        Thread(target=self.server_thread, daemon=True).start()
        Thread(target=self.auto_ping, daemon=True).start()

    def server_thread(self):
        while True:
            drone_socket, addr = self.server_socket.accept()
            client_ip = addr[0]
            drone_address = self.drone_addresses[client_ip]
            drone = self.gs.drones[drone_address]
            try:
                # Receiving data from the drone
                response_bytes = drone_socket.recv(1024)
                if response_bytes:
                    if len(response_bytes) > 1:
                        drone_response = DroneResponse(response_bytes)

                        # filter message for current session
                        if drone_response.session_id == self.session_id:
                            drone_response.print(drone.name)
                            drone.process_response(drone_response)
                        else:
                            print(f"{drone.name} sent a message to a previous session")
                    else:
                        print(f"{drone.name} doesn't have a response")

                    request_queue = self.drone_message_queues[drone_address]
                    if request_queue.empty():
                        drone_socket.sendall(bytes([0]))
                    else:
                        drone_request: DroneRequest = request_queue.get()
                        drone_socket.sendall(drone_request.bytes())
                else:
                    break
            finally:
                drone_socket.close()

    def auto_ping(self):

        while True:
            for address, drone in self.gs.drones.items():
                drone.check_connection()
                self.send_request(address, RequestType.Ping)
            time.sleep(4)

    def send_request(self, address, request_type, waypoints=None):
        print(f'Request queued for {self.gs.drones[address].name}')
        drone_request = DroneRequest(request_type, self.gs.server.session_id, self.pair_id, waypoints=waypoints)
        self.pair_id += 1
        self.drone_message_queues[address].put(drone_request)
