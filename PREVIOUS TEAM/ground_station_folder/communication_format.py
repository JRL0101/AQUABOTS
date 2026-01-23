from datetime import datetime
from enum import Enum
import struct
import tkinter as tk

import pytz


class RequestType(Enum):
    Ping = 0
    Start = 1
    Stop = 2
    Save = 3

class DroneMissionState(Enum):
    Init = 0
    Started = 1
    Stopped = 2

# Constants
REQUEST_SESSION_ID_POS = 0
REQUEST_PAIR_ID_POS = 4
REQUEST_TYPE_POS = 8
REQUEST_WAYPOINT_COUNT_POS = 9
REQUEST_WAYPOINTS_START_POS = 10
WAYPOINT_SIZE = 16  # Each waypoint consists of two doubles (latitude and longitude), 8 bytes each


class DroneRequest:
    def __init__(self, request_type, waypoints=None):
        self.request_type = request_type
        self.waypoints = waypoints

    def payload(self):
        # Process for Start request type
        if self.request_type == RequestType.Start:
            request_string = "start"
            for lat, lon in self.waypoints:
                request_string += "," + lat
                request_string += "," + lon

        if self.request_type == RequestType.Stop:
            request_string = "stop"

        if self.request_type == RequestType.Save:
            request_string = "save"

        return request_string

# Constants with 'RESPONSE_' prefix
RESPONSE_SESSION_ID_POS = 0
RESPONSE_PAIR_ID_POS = 4
RESPONSE_GPS_IS_WORKING_POS = 8
RESPONSE_COMPASS_IS_WORKING_POS = 9
RESPONSE_LATITUDE_POS = 10
RESPONSE_LONGITUDE_POS = 18
RESPONSE_HEADING_POS = 26
RESPONSE_SECOND_POS = 34
RESPONSE_MINUTE_POS = 35
RESPONSE_HOUR_POS = 36
RESPONSE_DAY_POS = 37
RESPONSE_MONTH_POS = 38
RESPONSE_YEAR_POS = 39
RESPONSE_STATE_POS = 40
RESPONSE_WAYPOINT_INDEX_POS = 41
RESPONSE_TEMPERATURE_POS = 42
#RESPONSE_PH_POS = 43
#RESPONSE_SALINITY_POS = 44
#RESPONSE_WIND_SPEED_POS = 45
#RESPONSE_WIND_DIRECTION_POS = 46


class DroneResponse:
    def __init__(self, response_bytes):
        self.session_id = struct.unpack_from('<I', response_bytes, RESPONSE_SESSION_ID_POS)[0]
        self.pair_id = struct.unpack_from('<I', response_bytes, RESPONSE_PAIR_ID_POS)[0]
        self.gps_is_working = bool(response_bytes[RESPONSE_GPS_IS_WORKING_POS])
        self.compass_is_working = bool(response_bytes[RESPONSE_COMPASS_IS_WORKING_POS])
        self.latitude = struct.unpack_from('d', response_bytes, RESPONSE_LATITUDE_POS)[0]
        self.longitude = struct.unpack_from('d', response_bytes, RESPONSE_LONGITUDE_POS)[0]
        self.heading = struct.unpack_from('d', response_bytes, RESPONSE_HEADING_POS)[0]
        self.second = response_bytes[RESPONSE_SECOND_POS]
        self.minute = response_bytes[RESPONSE_MINUTE_POS]
        self.hour = response_bytes[RESPONSE_HOUR_POS]
        self.day = response_bytes[RESPONSE_DAY_POS]
        self.month = response_bytes[RESPONSE_MONTH_POS]
        self.year = response_bytes[RESPONSE_YEAR_POS]
        self.state = DroneMissionState(response_bytes[RESPONSE_STATE_POS])
        self.waypoint_index = response_bytes[RESPONSE_WAYPOINT_INDEX_POS]
        self.temperature = response_bytes[RESPONSE_TEMPERATURE_POS]

    def print(self, drone_name):
        print(f"{drone_name} response:")
        print(f"  Session ID: {self.session_id}, Pair ID: {self.pair_id}")
        print(f"  GPS Working: {self.gps_is_working}, Compass Working: {self.compass_is_working}")
        print(f"  Latitude: {self.latitude}, Longitude: {self.longitude}, Heading: {self.heading}")
        print(f"  Time: {self.hour:02d}:{self.minute:02d}:{self.second:02d}, "
              f"Date: {self.day:02d}/{self.month:02d}/{self.year:02d}")
        print(f"  Planner State: {self.state.name}, Waypoint Index: {self.waypoint_index}")
        print(f"  Temperature: {self.temperature}")

    def report(self):
        response_str = f"State: {self.state.name}, Waypoint: {self.waypoint_index}\n"
        response_str += f"GPS: {sensor_status(self.gps_is_working)}, Compass: {sensor_status(self.compass_is_working)}\n"
        response_str += f"Temperature sensor: {sensor_status(self.temperature)}"
        return response_str


def sensor_status(is_working):
    return '✓' if is_working else '✗'


def utc_to_arizona_time(hour, minute, second):
    # Create a UTC datetime object for today with the given time
    utc_time = datetime.utcnow().replace(hour=hour, minute=minute, second=second, microsecond=0, tzinfo=pytz.utc)

    # Convert to Arizona time (MST, UTC-7)
    arizona_time = utc_time.astimezone(pytz.timezone('America/Phoenix'))

    # Format the time in a 12-hour format with AM/PM
    return arizona_time.strftime('%I:%M:%S %p')
