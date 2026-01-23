import time
from typing import Optional
import csv

from PIL import ImageTk
from tkintermapview.canvas_position_marker import CanvasPositionMarker

from typing import TYPE_CHECKING

from communication_format import *

if TYPE_CHECKING:
    from ground_station import GroundStation


SENSOR_DATA_TABLE_FILENAME = r"ground_station\output\sensor_data_table.csv"
SENSOR_DATA_TABLE_HEADER = ["Drone ID", "Latitude", "Longitude", "Temperature"]


def new_label(ribbon_frame, text, font):
    drone_label = tk.Label(ribbon_frame, text=text, font=font, anchor='w',
                           width=len(f"Last message: "), justify=tk.LEFT)
    drone_label.pack(side=tk.TOP, fill=tk.BOTH)
    return drone_label

class Drone:
    def __init__(self, _gs, _name: str, ribbon_frame):
        self.gs: GroundStation = _gs
        self.name: str = _name
        self.connection_label = None
        self.report_label = None
        self.last_message_time = 0
        self.marker: Optional[CanvasPositionMarker] = None

        new_label(ribbon_frame, self.name, ("Helvetica", 12, "bold"))
        self.connection_label = new_label(ribbon_frame, "Connection: Lost", ("Helvetica", 9))
        self.report_label = new_label(ribbon_frame, "-\n-\n-\n-\n-\n-\n-", ("Helvetica", 9))

    def check_connection(self):
        if (time.time() - self.last_message_time) > 5:
            self.update_connection(False)
            self.report_label.config(foreground="grey")
            self.delete_marker()

    def delete_marker(self):
        if self.marker is not None:
            self.marker.delete()
            self.marker.draw()

    def process_response(self, drone_response: DroneResponse):
        # create new marker
        self.delete_marker()
        if drone_response.gps_is_working:
            heading = drone_response.heading if drone_response.compass_is_working else 0
            original_image = ImageTk.PhotoImage(self.gs.file_manager.drone_image.rotate(-heading, expand=True))
            pos = drone_response.latitude, drone_response.longitude
            self.marker = self.gs.gui.map.set_marker(*pos, text=self.name, icon=original_image) #Update drone's location on the GUI

        self.last_message_time = time.time()
        self.update_connection(True)
        self.update_report(drone_response.report())

    def update_connection(self, connection_status):
        self.connection_label.config(text=f'Connection: {"Connected" if connection_status else "Lost"}')

    def update_report(self, text):
        self.report_label.config(text=text)
        self.report_label.config(foreground="black")

    #
    def process_sensor_data(self, pos, temp):
    #     """Process sensor data coming from drone into one marker and data point in the sensor data table"""
    #
    #     # add sensor data to sensor data table
         entry = [*pos, temp]
         if pos.path.exists(SENSOR_DATA_TABLE_FILENAME):
             with open(SENSOR_DATA_TABLE_FILENAME, mode="a", newline="") as file:
                 csv.writer(file).writerow(entry)
         else:
             with open(SENSOR_DATA_TABLE_FILENAME, mode="w", newline="") as file:
                 csv.writer(file).writerow(SENSOR_DATA_TABLE_HEADER)
                 csv.writer(file).writerow(entry)