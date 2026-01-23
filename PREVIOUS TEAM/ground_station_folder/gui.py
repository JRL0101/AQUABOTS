from typing import TYPE_CHECKING, Optional

from communication_format import DroneRequest, RequestType

import tkintermapview
import tkinter as tk
import time

from tkintermapview.canvas_path import CanvasPath
from tkintermapview.canvas_position_marker import CanvasPositionMarker

if TYPE_CHECKING:
    from ground_station import GroundStation


class GUI:
        
    def __init__(self, _gs):
        self.gs: GroundStation = _gs

        # Create frames
        self.ribbon_frame = tk.Frame(self.gs.root)
        self.ribbon_frame.pack(side=tk.RIGHT, fill="both")

        # Create start/stop/save buttons
        top_frame = tk.Frame(self.ribbon_frame)
        top_frame.pack(side=tk.TOP, fill="x")
        tk.Button(self.ribbon_frame, text="Start", command=self.start).pack(side=tk.TOP, fill="x")
        tk.Button(self.ribbon_frame, text="Stop", command=self.stop).pack(side=tk.TOP, fill="x")
        tk.Button(self.ribbon_frame, text="Save", command=self.save).pack(side=tk.TOP, fill="x")

        self.mission_waypoints: list[CanvasPositionMarker] = []
        self.mission_waypoints_path: Optional[CanvasPath] = None

        # create map with default view
        self.map = tkintermapview.TkinterMapView(self.gs.root, width=1000, height=750, corner_radius=0)
        self.map.pack(side=tk.LEFT)
        self.map.set_zoom(20)

        # add command to add waypoints
        self.skip_click = False
        self.map.add_left_click_map_command(self.add_mission_waypoint)

        # add waypoints from file_manager file
        for mission_waypoint in self.gs.file_manager.config.get("mission_waypoints", []):
            self.map.set_position(*mission_waypoint)
            self.add_mission_waypoint(mission_waypoint)

        # update map display
        self.map.set_position(32.234493, -110.95102)

    def start(self):
        time.sleep(5) # gives the Raspberry Pi time to determine its position before it receives any new waypoint instructions
        mission_waypoints = self.mission_waypoint_positions()
        startRequest = DroneRequest(RequestType.Start, mission_waypoints)
        self.gs.client.publish('raspberry/GroundStation', payload=startRequest.payload(), qos=2, retain=False)

    def stop(self):
        stopRequest = DroneRequest(RequestType.Stop)
        self.gs.client.publish('raspberry/GroundStation', payload=stopRequest.payload(), qos=2, retain=False)

    def save(self):
        saveRequest = DroneRequest(RequestType.Save)
        self.gs.client.publish('raspberry/GroundStation', payload=saveRequest.payload(), qos=2, retain=False)
        self.gs.file_manager.save(self.mission_waypoint_positions())

    def mission_waypoint_positions(self):
        return [mission_waypoint.position for mission_waypoint in self.mission_waypoints]

    def add_mission_waypoint(self, coords):
        # avoid multiple clicks
        if self.skip_click:
            self.skip_click = False
            return

        # add button with ability to delete by left click
        left_func = self.delete_mission_waypoint
        new_mission_waypoint = self.map.set_marker(*coords, icon=self.gs.file_manager.waypoint_icon, command=left_func)

        # add waypoint to mission waypoint list and update map
        self.mission_waypoints.append(new_mission_waypoint)
        self.update_map()

    def delete_mission_waypoint(self, marker: CanvasPositionMarker):
        # mark as a click so multiple clicks are avoided
        self.skip_click = True

        # delete waypoint visual, remove from list and update map
        marker.delete()
        self.mission_waypoints.remove(marker)
        self.update_map()

    def update_map(self):
        # update name of waypoints
        for n, mission_waypoint in enumerate(self.mission_waypoints):
            mission_waypoint.text = f"{n}"
            mission_waypoint.draw()

        # update waypoint path
        if self.mission_waypoints_path is not None:
            self.mission_waypoints_path.delete()
        if len(self.mission_waypoints) > 1:
            self.mission_waypoints_path = self.map.set_path(self.mission_waypoint_positions())


