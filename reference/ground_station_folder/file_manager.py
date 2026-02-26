import json
import os
from typing import TYPE_CHECKING
from PIL import ImageTk, Image

if TYPE_CHECKING:
    from ground_station import GroundStation


class FileManager:
    def __init__(self, _gs):
        self.gs: GroundStation = _gs
        
        # Get the base directory path
        base_dir = os.path.dirname(os.path.abspath(__file__))
        graphics_dir = os.path.join(base_dir, "graphics")
        
        # Create graphics directory if it doesn't exist
        os.makedirs(graphics_dir, exist_ok=True)
        
        # Default configuration
        self.default_config = {
            "drones": [],
            "mission_waypoints": []
        }
        
        # Initialize with default values
        self.dummy_icon = self._create_placeholder_icon(15)
        self.waypoint_icon = self._create_placeholder_icon(10)
        self.target_waypoint_icon = None
        self.drone_image = None
        self.config = self.default_config.copy()
        
        # Try to load images
        try:
            self.dummy_icon = self._load_image(os.path.join(graphics_dir, "waypoint_icon.png"), 15)
            self.waypoint_icon = self._load_image(os.path.join(graphics_dir, "waypoint_icon.png"), 10)
            self.target_waypoint_icon = self._load_image(os.path.join(graphics_dir, "waypoint_selected_icon.png"))
            self.drone_image = self._load_image(os.path.join(graphics_dir, "drone.png"), 10)
        except Exception as e:
            print(f"Warning: Could not load some image files: {str(e)}")

        # Load or create config file
        self.config_filename = os.path.join(base_dir, "ground_station.conf")
        self._load_config()

    def _load_image(self, path, reduce_factor=None):
        """Helper method to load an image with optional resizing"""
        if not os.path.exists(path):
            raise FileNotFoundError(f"Image file not found: {path}")
            
        img = Image.open(path)
        if reduce_factor:
            img = img.reduce(reduce_factor)
        return img

    def _create_placeholder_icon(self, size=10):
        """Create a simple placeholder icon if real images can't be loaded"""
        from PIL import ImageDraw
        img = Image.new('RGBA', (size*2, size*2), (255, 255, 255, 0))
        draw = ImageDraw.Draw(img)
        draw.ellipse((0, 0, size*2-1, size*2-1), outline='red', width=1)
        return ImageTk.PhotoImage(img)

    def _load_config(self):
        """Load or create config file"""
        try:
            if os.path.exists(self.config_filename):
                with open(self.config_filename, "r") as file:
                    loaded_config = json.load(file)
                    # Merge with default config to ensure all keys exist
                    self.config = {**self.default_config, **loaded_config}
            else:
                print(f"Config file not found, creating default at {self.config_filename}")
                self.save(self.config["mission_waypoints"])
        except Exception as e:
            print(f"Error loading config: {str(e)}, using default configuration")
            self.config = self.default_config.copy()

    def save(self, mission_waypoints):
        """Save mission waypoints to config file"""
        self.config["mission_waypoints"] = mission_waypoints
        try:
            with open(self.config_filename, 'w') as file:
                json.dump(self.config, file, indent=4)
        except Exception as e:
            print(f"Error saving config: {str(e)}")
            raise