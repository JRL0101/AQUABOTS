import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkintermapview import TkinterMapView
from mqtt_handler import MQTTHandler
from datetime import datetime
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from PIL import Image, ImageTk

class DroneControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Control Station")
        self.root.geometry("1500x800")
        
        # Connection monitoring
        self.last_update_time = None
        self.connection_monitor_active = True
        self.connection_check_interval = 5  # seconds
        
        # Temperature graph data
        self.temperature_history = []
        self.time_history = []
        self.gps_history = []  # Stores (lat, lon) tuples
        self.max_points = 50
        self.last_valid_gps = (None, None)  # (lat, lon)
        
        # Setup MQTT
        self.mqtt = MQTTHandler()
        self.mqtt.gui_update_callback = self.update_sensor_display
        self.mqtt.map_update_callback = self.update_map
        self.mqtt.connect()
        
        # Start connection monitoring thread
        self.start_connection_monitor()
        
        # UI Configuration
        self.setup_styles()
        self.setup_frames()
        self.setup_left_panel()
        self.setup_map()
        self.setup_status_box()
        self.setup_sensor_display()
        self.setup_gps_input()
        self.setup_control_buttons()
        self.start_blinking_indicators()
        root.after(100, self.start_blinking_indicators) 
    def emergency_stop(self):
        """Handle emergency stop"""
        response = messagebox.askyesno(
            "Confirm Emergency Stop",
            "Are you sure you want to end operations?\nThis will immediately terminate all drone processes.",
            icon='warning'
        )
        if response:
            self.mqtt.send_command("emergency", "shutdown")
            self.drone_status_var.set("EMERGENCY STOP SENT")
            self.drone_status_label.config(fg="#FF5252")  # Red for emergency
            self.emergency_btn.config(state=tk.DISABLED, bg='#9a0007')

    def setup_control_buttons(self):
        """Create control buttons in bottom right corner"""
        # Create a container frame for the buttons at the bottom
        button_container = tk.Frame(self.right_frame, bg=self.bg_color)
        button_container.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)
        
        # Add padding frame to push buttons to the right
        padding_frame = tk.Frame(button_container, bg=self.bg_color)
        padding_frame.pack(side=tk.RIGHT)
        
        # Save Data button (topmost)
        self.save_btn = tk.Button(
            padding_frame,
            text="SAVE DATA",
            command=self.save_sensor_data,
            **self.button_styles['save']
        )
        self.save_btn.pack(fill=tk.X, pady=5)
        
        # Pause Operations button (middle)
        self.stop_btn = tk.Button(
            padding_frame,
            text="PAUSE OPERATIONS",
            command=self.stop_operations,
            **self.button_styles['stop']
        )
        self.stop_btn.pack(fill=tk.X, pady=5)
        
        # Emergency Stop button (bottom)
        self.emergency_btn = tk.Button(
            padding_frame,
            text="EMERGENCY STOP",
            command=self.emergency_stop,  # Now this method exists
            **self.button_styles['emergency']
        )
        self.emergency_btn.pack(fill=tk.X, pady=5)
    
    def start_blinking_indicators(self):
        self.blink_state = False
        """Initialize blinking for error indicators"""
        self.blink_indicators()
    def blink_indicators(self):
        """Toggle blink state for error indicators"""
        # Only proceed if we have valid images and canvas
        if (hasattr(self, 'boat_img_tk')) and self.component_canvas.winfo_ismapped():
            self.blink_state = not self.blink_state
        
            # Only redraw if we have active errors
            if (self.mqtt.temp_error_state() or self.mqtt.imu_error_state() or self.mqtt.gps_error_state()):
                self.resize_component_images()
    
        # Schedule next blink (500ms interval)
        self.root.after(400, self.blink_indicators)
    def update_map(self, lat, lon):
            """Update drone position on map"""
            if hasattr(self, 'drone_marker') and hasattr(self, 'map_widget'):
                self.drone_marker.set_position(lat, lon)
                self.map_widget.set_position(lat, lon)
    def setup_left_panel(self):
        """Create left panel with top and bottom sections"""
        # Top section - Component Conditions with layered images
        component_frame = tk.LabelFrame(
            self.left_spacer,
            text=" Component Conditions ",
            bg=self.bg_color,
            fg=self.text_color,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=10,
            height=350
        )
        component_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(10, 5))
        
        # Create a canvas for layering images
        self.component_canvas = tk.Canvas(
            component_frame,
            bg=self.bg_color,
            highlightthickness=0
        )
        self.component_canvas.pack(fill=tk.BOTH, expand=True)
        
        try:
            # Load all images using PIL for better resizing
            boat_img = Image.open("graphics/boat outline.png")
            IMUG_img = Image.open("graphics/IMUG.png")
            tempg_img = Image.open("graphics/TEMPG.png")
            gpsg_img = Image.open("graphics/GPSG.png")
            IMUB_img = Image.open("graphics/IMUB.png")
            tempB_img = Image.open("graphics/TEMPB.png")
            gpsB_img = Image.open("graphics/GPSB.png")
            # Store original images
            self.boat_img_pil = boat_img
            self.IMUG_img_pil = IMUG_img
            self.tempg_img_pil = tempg_img
            self.gpsg_img_pil = gpsg_img
            self.IMUB_img_pil = IMUB_img
            self.tempB_img_pil = tempB_img
            self.gpsB_img_pil = gpsB_img
            # Initial placeholder for Tkinter images
            self.boat_img_tk = None
            self.IMUG_img_tk = None
            self.tempg_img_tk = None
            self.gpsg_img_tk = None
            self.IMUB_img_tk = None
            self.tempB_img_tk = None
            self.gpsB_img_tk = None
            # Make the canvas resize properly
            self.component_canvas.bind("<Configure>", lambda e: self.resize_component_images())
            
        except Exception as e:
            print(f"Error loading component images: {e}")
            # Fallback if images can't be loaded
            self.component_canvas.create_text(
                150, 150,
                text="Component Status Indicators",
                fill=self.text_color,
                font=('Arial', 12)
            )
        
        # Bottom section (temperature graph)
        self.setup_temperature_graph()

    def resize_component_images(self):
        """Handle canvas resizing with proper error state indicators"""
        if not hasattr(self, 'boat_img_pil'):
            return
    
        canvas_width = self.component_canvas.winfo_width()
        canvas_height = self.component_canvas.winfo_height()

        # Skip if canvas isn't ready
        if canvas_width <= 10 or canvas_height <= 10:
            return
    
        # Clear existing images
        self.component_canvas.delete("all")

        # Boat image calculations
        max_boat_height = max(10, canvas_height - 40)
        max_boat_width = max(10, canvas_width - 40)
        boat_scale = min(max_boat_height / self.boat_img_pil.height,max_boat_width / self.boat_img_pil.width,
        1.0
    )
        boat_width = max(10, int(self.boat_img_pil.width * boat_scale))
        boat_height = max(10, int(self.boat_img_pil.height * boat_scale))

        # Create and position boat image
        boat_img_resized = self.boat_img_pil.resize((boat_width, boat_height), Image.LANCZOS)
        self.boat_img_tk = ImageTk.PhotoImage(boat_img_resized)
        boat_x = (canvas_width - boat_width) // 2
        boat_y = (canvas_height - boat_height) // 2
        self.component_canvas.create_image(boat_x, boat_y, image=self.boat_img_tk, anchor=tk.NW)

        # Calculate indicator size (20% of boat height)
        indicator_height = max(10, int(boat_height * 0.2))

        # Resize all indicator images
        def resize_img(img_pil, height):
            aspect = img_pil.width / img_pil.height
            width = max(10, int(height * aspect))
            return img_pil.resize((width, height), Image.LANCZOS)

        # Resize and store all images
        self.tempg_img_tk = ImageTk.PhotoImage(resize_img(self.tempg_img_pil, indicator_height))
        self.tempb_img_tk = ImageTk.PhotoImage(resize_img(self.tempB_img_pil, indicator_height))
        self.imug_img_tk = ImageTk.PhotoImage(resize_img(self.IMUG_img_pil, indicator_height))
        self.imub_img_tk = ImageTk.PhotoImage(resize_img(self.IMUB_img_pil, indicator_height))
        self.gpsg_img_tk = ImageTk.PhotoImage(resize_img(self.gpsg_img_pil, indicator_height))
        self.gpsb_img_tk = ImageTk.PhotoImage(resize_img(self.gpsB_img_pil, indicator_height))

        # Position calculations
        left_offset = boat_x - 85
        right_offset = boat_x + boat_width - int(indicator_height * (self.gpsg_img_pil.width/self.gpsg_img_pil.height)) + 85

        # Temperature indicator
        if self.mqtt.temp_error_state():
            if self.blink_state:
                self.component_canvas.create_image(left_offset, boat_y + 70, image=self.tempb_img_tk, anchor=tk.NW)
        else:
            self.component_canvas.create_image(left_offset, boat_y + 70, image=self.tempg_img_tk, anchor=tk.NW)

        # IMU indicator - fixed to properly show error state
        if hasattr(self.mqtt, 'imu_error_state') and self.mqtt.imu_error_state():
            if self.blink_state:
                self.component_canvas.create_image(left_offset, boat_y + 90 + indicator_height + 10, image=self.imub_img_tk, anchor=tk.NW)
        else:
            self.component_canvas.create_image(left_offset, boat_y + 90 + indicator_height + 10, image=self.imug_img_tk, anchor=tk.NW)

        # GPS indicator
        if self.mqtt.gps_error_state():
            if self.blink_state:
                self.component_canvas.create_image(right_offset, boat_y + (boat_height//2) - (indicator_height//2), image=self.gpsb_img_tk, anchor=tk.NW)
        else:
            self.component_canvas.create_image(right_offset, boat_y + (boat_height//2) - (indicator_height//2), image=self.gpsg_img_tk, anchor=tk.NW)

    def setup_temperature_graph(self):
        """Create temperature graph frame in bottom half of left panel"""
        graph_frame = tk.LabelFrame(
            self.left_spacer,
            bg=self.bg_color,
            fg=self.text_color,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=10,
            height=350
        )
        graph_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(5, 10))
        
        # Create matplotlib figure
        self.temp_fig = Figure(figsize=(5, 2.5), dpi=80, facecolor=self.frame_color)
        self.temp_ax = self.temp_fig.add_subplot(111)
        self.temp_ax.set_facecolor(self.frame_color)
        
        # Style the plot
        self.temp_ax.tick_params(colors=self.text_color)
        for spine in self.temp_ax.spines.values():
            spine.set_color(self.text_color)
        self.temp_ax.set_xlabel('Time (s)', color=self.text_color)
        self.temp_ax.set_ylabel('°C', color=self.text_color, rotation=0, va='bottom')
        self.temp_ax.yaxis.set_label_coords(-0.1, 1.02)  # Adjust position
        self.temp_ax.set_title('Temperature Over Time', color=self.text_color)
        
        # Create canvas and add to frame
        self.temp_canvas = FigureCanvasTkAgg(self.temp_fig, master=graph_frame)
        self.temp_line, = self.temp_ax.plot([], [], 'r-')  # Red line
        self.temp_points = self.temp_ax.plot([], [], 'bo', markersize=4, alpha=0.7)[0]  # GPS points
        self.temp_annotations = []
        self.temp_canvas.draw()
        self.temp_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def update_temperature_graph(self):
        """Update the temperature graph with new data"""
        if len(self.temperature_history) == 0:
            return
        
        # Clear previous annotations
        for ann in self.temp_annotations:
            ann.remove()
        self.temp_annotations = []
        
        # Convert timestamps to relative seconds for x-axis
        time_deltas = [(t - self.time_history[0]).total_seconds() for t in self.time_history]
        
        # Update plot data
        self.temp_line.set_data(time_deltas, self.temperature_history)
        self.temp_points.set_data(time_deltas, self.temperature_history)
        
        # Add annotations for GPS coordinates
        for i, (t, temp, gps) in enumerate(zip(time_deltas, self.temperature_history, self.gps_history)):
            if gps != (None, None):
                lat, lon = gps
                ann = self.temp_ax.annotate(
                    f"Lat: {lat:.4f}\nLon: {lon:.4f}",
                    xy=(t, temp),
                    xytext=(5, 5),
                    textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.5', fc='white', alpha=0.7),
                    fontsize=8
                )
                self.temp_annotations.append(ann)
        
        # Adjust axes
        self.temp_ax.relim()
        self.temp_ax.autoscale_view()
        
        # Redraw
        self.temp_canvas.draw()

    def start_connection_monitor(self):
        """Start a thread to monitor connection status"""
        def monitor():
            while self.connection_monitor_active:
                if self.last_update_time and (time.time() - self.last_update_time > self.connection_check_interval):
                    self.root.after(0, self.update_connection_status, False)
                time.sleep(1)
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
    
    def update_connection_status(self, connected):
        """Update the connection status display with color coding"""
        if connected:
            self.mqtt_status_var.set("Connected")
            self.drone_status_var.set("Receiving Data")
            # Set colors for connected state (green)
            self.mqtt_status_label.config(fg="#4CAF50")
            self.drone_status_label.config(fg="#4CAF50")
            # Re-enable controls when connection is restored
            if not self.nav_btn['state'] == tk.DISABLED:  # Only enable if not in navigation mode
                self.nav_btn.config(state='normal')
            self.save_btn.config(state='normal')
            self.stop_btn.config(state='normal')
            self.emergency_btn.config(state='normal')
        else:
            self.mqtt_status_var.set("Connection Lost")
            self.drone_status_var.set("Connection Lost")
            # Set colors for disconnected state (red)
            self.mqtt_status_label.config(fg="#FF5252")
            self.drone_status_label.config(fg="#FF5252")
            # Disable controls when connection is lost
            self.nav_btn.config(state='disabled')
            self.save_btn.config(state='disabled')
            self.stop_btn.config(state='disabled')

    def setup_styles(self):
        """Configure dark theme styles"""
        self.bg_color = '#2d2d2d'
        self.frame_color = '#3d3d3d'
        self.text_color = 'white'
        self.root.configure(bg=self.bg_color)
        
        # Custom button styles
        button_style = {
            'font': ('Arial', 11),
            'height': 2,
            'width': 20,
            'relief': tk.FLAT,
            'bd': 0
        }
        self.button_styles = {
            'emergency': {'bg': '#d32f2f', 'fg': 'white', **button_style},
            'stop': {'bg': '#FFA000', 'fg': 'white', **button_style},
            'save': {'bg': '#388E3C', 'fg': 'white', **button_style},
            'nav': {'bg': '#1976D2', 'fg': 'white', **button_style}
        }
    
    def setup_frames(self):
        """Create main application frames with adjusted spacing"""
        # Main container frame
        self.main_container = tk.Frame(self.root, bg=self.bg_color)
        self.main_container.pack(fill=tk.BOTH, expand=True)
        
        # Left spacer (twice the width of right spacer)
        self.left_spacer = tk.Frame(self.main_container, width=500, bg=self.bg_color)
        self.left_spacer.pack(side=tk.LEFT, fill=tk.Y)
        
        # Middle frame for map
        self.map_frame = tk.Frame(self.main_container, bg='white')
        self.map_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Right frame for controls
        self.right_frame = tk.Frame(self.main_container, width=400, bg=self.bg_color)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Right spacer (half the width of left spacer)
        self.right_spacer = tk.Frame(self.main_container, width=20, bg=self.bg_color)
        self.right_spacer.pack(side=tk.RIGHT, fill=tk.Y)
    
    def setup_map(self):
        """Initialize map widget"""
        self.map_widget = TkinterMapView(
            self.map_frame, 
            width=700, 
            height=700,
            corner_radius=0
        )
        self.map_widget.pack(fill=tk.BOTH, expand=True)
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        self.map_widget.set_position(32.23299, -110.95205)
        self.map_widget.set_zoom(15)
        
        # Add drone marker
        self.drone_marker = self.map_widget.set_marker(
            32.23299, -110.95205,
            text="Eddy III",
            marker_color_circle="red",
            text_color="black"
        )
    
    def setup_status_box(self):
        """Create system status frame with color-coded status labels"""
        status_frame = tk.LabelFrame(
            self.right_frame,
            text=" System Status ",
            bg=self.bg_color,
            fg=self.text_color,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=10
        )
        
        # Connection status
        tk.Label(status_frame, text="MQTT Status:", 
                bg=self.bg_color, fg=self.text_color).grid(row=0, column=0, sticky='w', pady=2)
        self.mqtt_status_var = tk.StringVar(value="Connecting...")
        self.mqtt_status_label = tk.Label(status_frame, textvariable=self.mqtt_status_var, 
                bg=self.bg_color, fg="white")  # White for connecting state
        self.mqtt_status_label.grid(row=0, column=1, sticky='w', pady=2)
        
        # Drone status
        tk.Label(status_frame, text="Drone Status:", 
                bg=self.bg_color, fg=self.text_color).grid(row=1, column=0, sticky='w', pady=2)
        self.drone_status_var = tk.StringVar(value="Waiting for connection")
        self.drone_status_label = tk.Label(status_frame, textvariable=self.drone_status_var, 
                bg=self.bg_color, fg="white")  # White for waiting state
        self.drone_status_label.grid(row=1, column=1, sticky='w', pady=2)
        
        status_frame.pack(fill=tk.X, padx=10, pady=10)
    
    def setup_sensor_display(self):
        """Create sensor readings frame"""
        readings_frame = tk.LabelFrame(
            self.right_frame,
            text=" Last Sensor Readings ",
            bg=self.bg_color,
            fg=self.text_color,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=10
        )
        
        # Temperature
        tk.Label(readings_frame, text="Temperature:", 
                bg=self.bg_color, fg=self.text_color).grid(row=0, column=0, sticky='w', pady=2)
        self.temp_var = tk.StringVar(value="N/A")
        tk.Label(readings_frame, textvariable=self.temp_var, 
                bg=self.bg_color, fg=self.text_color).grid(row=0, column=1, sticky='w', pady=2)
        
        # IMU
        #tk.Label(readings_frame, text="IMU:", 
                #bg=self.bg_color, fg=self.text_color).grid(row=1, column=0, sticky='w', pady=2)
        #self.imu_var = tk.StringVar(value="N/A")
        #tk.Label(readings_frame, textvariable=self.imu_var, 
                #bg=self.bg_color, fg=self.text_color).grid(row=1, column=1, sticky='w', pady=2)
        
        # GPS
        tk.Label(readings_frame, text="GPS Coordinates:", 
                bg=self.bg_color, fg=self.text_color).grid(row=2, column=0, sticky='w', pady=2)
        self.gps_var = tk.StringVar(value="N/A")
        tk.Label(readings_frame, textvariable=self.gps_var, 
                bg=self.bg_color, fg=self.text_color).grid(row=2, column=1, sticky='w', pady=2)
        
        readings_frame.pack(fill=tk.X, padx=10, pady=10)
    
    def setup_gps_input(self):
        """Create GPS input frame for navigation"""
        gps_frame = tk.LabelFrame(
            self.right_frame,
            text=" GPS Input ",
            bg=self.bg_color,
            fg=self.text_color,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=10
        )
        
        # Latitude input
        tk.Label(gps_frame, text="Latitude:", 
               bg=self.bg_color, fg=self.text_color).grid(row=0, column=0, sticky='w', pady=2)
        self.lat_entry = tk.Entry(gps_frame, bg='white', fg='black')
        self.lat_entry.grid(row=0, column=1, sticky='ew', pady=2)
        
        # Longitude input
        tk.Label(gps_frame, text="Longitude:", 
               bg=self.bg_color, fg=self.text_color).grid(row=1, column=0, sticky='w', pady=2)
        self.lon_entry = tk.Entry(gps_frame, bg='white', fg='black')
        self.lon_entry.grid(row=1, column=1, sticky='ew', pady=2)
        
        # Start Navigation button
        self.nav_btn = tk.Button(
            gps_frame,
            text="START NAVIGATION",
            command=self.start_navigation,
            **self.button_styles['nav']
        )
        self.nav_btn.grid(row=2, column=0, columnspan=2, sticky='ew', pady=(10, 0))
        
        gps_frame.pack(fill=tk.X, padx=10, pady=10)
    
    def setup_control_buttons(self):
        """Create control buttons in bottom right corner"""
        # Create a container frame for the buttons at the bottom
        button_container = tk.Frame(self.right_frame, bg=self.bg_color)
        button_container.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)
        
        # Add padding frame to push buttons to the right
        padding_frame = tk.Frame(button_container, bg=self.bg_color)
        padding_frame.pack(side=tk.RIGHT)
        
        # Save Data button (topmost)
        self.save_btn = tk.Button(
            padding_frame,
            text="SAVE DATA",
            command=self.save_sensor_data,
            **self.button_styles['save']
        )
        self.save_btn.pack(fill=tk.X, pady=5)
        
        # Pause Operations button (middle)
        self.stop_btn = tk.Button(
            padding_frame,
            text="PAUSE OPERATIONS",
            command=self.stop_operations,
            **self.button_styles['stop']
        )
        self.stop_btn.pack(fill=tk.X, pady=5)
        
        # Emergency Stop button (bottom)
        self.emergency_btn = tk.Button(
            padding_frame,
            text="EMERGENCY STOP",
            command=self.emergency_stop,
            **self.button_styles['emergency']
        )
        self.emergency_btn.pack(fill=tk.X, pady=5)
    
    def start_navigation(self):
        """Handle navigation start with GPS coordinates"""
        try:
            lat = float(self.lat_entry.get())
            lon = float(self.lon_entry.get())
            
            if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                raise ValueError("Invalid coordinate range")
                
            confirm = messagebox.askyesno(
                "Confirm Navigation",
                f"Navigate to:\nLatitude: {lat}\nLongitude: {lon}",
                icon='question'
            )
            
            if confirm:
                # Send navigation command to Raspberry Pi
                self.mqtt.send_command("navigation", {"lat": lat, "lon": lon})
                self.drone_status_var.set("Navigating to target")
                self.drone_status_label.config(fg="#4CAF50")  # Green for active navigation
                
                # Disable inputs and navigation button
                self.lat_entry.config(state='disabled')
                self.lon_entry.config(state='disabled')
                self.nav_btn.config(state='disabled')  # Lock navigation button
                
        except ValueError as e:
            messagebox.showerror("Invalid Input", "Please enter valid GPS coordinates\nLatitude: -90 to 90\nLongitude: -180 to 180")
    
    def stop_operations(self):
        """Send normal stop command and re-enable GPS inputs"""
        self.mqtt.send_command("control", "stop")
        self.drone_status_var.set("STOP COMMAND SENT")
        self.drone_status_label.config(fg="white")  # Reset to white
        
        # Re-enable GPS inputs and navigation button
        self.lat_entry.config(state='normal')
        self.lon_entry.config(state='normal')
        self.nav_btn.config(state='normal')  # Unlock navigation button
    
    def update_sensor_display(self, data):
        """Update sensor display with new data"""
        #print("Received Data:", data)
        self.last_update_time = time.time()  # Update last received message time
        self.update_connection_status(True)  # Confirm connection is active
    
        sensor_type = data['sensor']
        value = data['value']
    
        if sensor_type == "temperature":
            if value == self.mqtt.TEMP_ERROR:
                last_valid = self.mqtt.get_last_temp_reading()
                # Only show "ERROR" if we have no valid readings yet
                if last_valid == self.mqtt.TEMP_ERROR:
                    display = "ERROR"
                else:
                    display = f"ERROR (Last: {last_valid}°C)"
                self.temp_var.set(display)
            else:
                display = f"{value}°C"
                self.temp_var.set(display)
                # Only graph if not error and we have GPS data
                if value != self.mqtt.TEMP_ERROR and self.last_valid_gps != (None, None):
                    self.temperature_history.append(float(value))
                    self.time_history.append(datetime.now())
                    self.gps_history.append(self.last_valid_gps)
                
                    # Keep only the most recent points
                    if len(self.temperature_history) > self.max_points:
                        self.temperature_history.pop(0)
                        self.time_history.pop(0)
                        self.gps_history.pop(0)
                
                    # Update the graph
                    self.update_temperature_graph()
    
        #elif sensor_type == "imu":
            #if value == self.mqtt.IMU_ERROR:
                #last_valid = self.mqtt.get_last_imu_reading()
                #if last_valid == self.mqtt.IMU_ERROR:
                    #display = "ERROR"
                #else:
                    #display = f"ERROR (Last: {last_valid})"
                #self.imu_var.set(display)
            #else:
                #display = str(value)
                #self.imu_var.set(display)
    
        elif sensor_type == "gps":
            #print("GPS Value Received:", value)  # Debug line
            lat = value.get("lat") if isinstance(value, dict) else None
            lon = value.get("lon") if isinstance(value, dict) else None
            #print(f"Extracted Lat/Lon: {lat}, {lon}")  # Debug line
        
            if lat is None or lat == self.mqtt.GPS_ERROR:
                last_valid = self.mqtt.get_last_gps_reading()
                if last_valid and last_valid['lat'] != self.mqtt.GPS_ERROR:
                    display = f"ERROR (Last: {last_valid['lat']:.6f}, {last_valid['lon']:.6f})"
                    self.last_valid_gps = (last_valid['lat'], last_valid['lon'])
                else:
                    display = "ERROR"
                    self.last_valid_gps = (None, None)
                self.gps_var.set(display)
            else:
                display = f"{lat:.6f}, {lon:.6f}"
                self.gps_var.set(display)
                self.last_valid_gps = (lat, lon)
    
    def save_sensor_data(self):
        """Save collected data to file"""
        data = self.mqtt.get_sensor_data()
        if not data:
            messagebox.showwarning("No Data", "No sensor data has been collected yet.")
            return
            
        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            initialfile=f"drone_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        )
        
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    f.write("Timestamp\tSensor\tValue\n")
                    for entry in data:
                        value = entry['value']
                        if isinstance(value, dict):  # GPS data
                            value = f"{value.get('lat', 'N/A')}, {value.get('lon', 'N/A')}"
                        f.write(f"{entry['timestamp']}\t{entry['sensor']}\t{value}\n")
                
                messagebox.showinfo("Success", f"Data saved to:\n{file_path}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save file:\n{str(e)}")

    def on_close(self):
        """Handle window closing"""
        self.connection_monitor_active = False
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
