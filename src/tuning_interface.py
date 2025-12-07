import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import time
import json
from http.server import HTTPServer, BaseHTTPRequestHandler
import struct

class RobotTuningInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("NxP Cup Robot Tuning Interface")
        self.root.geometry("800x600")
        self.root.configure(bg='#2b2b2b')
        
        # Connection settings
        self.esp_ip = "192.168.4.1"
        self.esp_port = 8080
        self.connected = False
        
        # Current values
        self.right_velocity = tk.DoubleVar(value=0.0)
        self.left_velocity = tk.DoubleVar(value=0.0)
        self.distance = tk.DoubleVar(value=0.0)
        
        # PID parameters for RIGHT motor velocity control
        self.right_vel_kp = tk.DoubleVar(value=1.0)
        self.right_vel_ki = tk.DoubleVar(value=0.0)
        self.right_vel_kd = tk.DoubleVar(value=0.0)
        
        # PID parameters for LEFT motor velocity control
        self.left_vel_kp = tk.DoubleVar(value=1.0)
        self.left_vel_ki = tk.DoubleVar(value=0.0)
        self.left_vel_kd = tk.DoubleVar(value=0.0)
        
        # Distance mode control
        self.distance_mode = tk.BooleanVar(value=False)
        
        # PID parameters for steering control
        self.steer_kp = tk.DoubleVar(value=1.0)
        self.steer_ki = tk.DoubleVar(value=0.0)
        self.steer_kd = tk.DoubleVar(value=0.0)
        
        # CubeMonitor integration
        self.cubemonitor_enabled = tk.BooleanVar(value=False)
        self.cubemonitor_clients = []  # WebSocket clients
        self.cubemonitor_server = None
        self.latest_data = {}
        
        self.setup_ui()
        
    def setup_ui(self):
        # Title
        title_label = tk.Label(self.root, text="Robot Tuning Interface", 
                              font=("Arial", 16, "bold"), 
                              bg='#2b2b2b', fg='white')
        title_label.pack(pady=10)
        
        # Connection frame
        conn_frame = tk.Frame(self.root, bg='#2b2b2b')
        conn_frame.pack(pady=5)
        
        tk.Label(conn_frame, text="ESP32 IP:", bg='#2b2b2b', fg='white').pack(side=tk.LEFT)
        self.ip_entry = tk.Entry(conn_frame, width=15)
        self.ip_entry.insert(0, self.esp_ip)
        self.ip_entry.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = tk.Button(conn_frame, text="Connect", 
                                   command=self.toggle_connection,
                                   bg='#4CAF50', fg='white')
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.status_label = tk.Label(conn_frame, text="Disconnected", 
                                   bg='#2b2b2b', fg='red')
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        # CubeMonitor frame
        cube_frame = tk.Frame(self.root, bg='#2b2b2b')
        cube_frame.pack(pady=5)
        
        cube_check = tk.Checkbutton(cube_frame, text="Enable STM32CubeMonitor", 
                                   variable=self.cubemonitor_enabled,
                                   command=self.toggle_cubemonitor,
                                   bg='#2b2b2b', fg='white',
                                   selectcolor='#2b2b2b')
        cube_check.pack(side=tk.LEFT)
        
        self.cube_status_label = tk.Label(cube_frame, text="Server: OFF", 
                                   bg='#2b2b2b', fg='gray')
        self.cube_status_label.pack(side=tk.LEFT, padx=10)
        
        # Auto-send checkbox
        auto_frame = tk.Frame(self.root, bg='#2b2b2b')
        auto_frame.pack(pady=5)
        
        self.auto_send = tk.BooleanVar()
        auto_check = tk.Checkbutton(auto_frame, text="Auto-send on slider change", 
                                   variable=self.auto_send,
                                   bg='#2b2b2b', fg='white',
                                   selectcolor='#2b2b2b')
        auto_check.pack()
        
        # Main control frame
        control_frame = tk.Frame(self.root, bg='#2b2b2b')
        control_frame.pack(pady=10, padx=20, fill='both', expand=True)
        
        # Right Velocity Slider
        self.create_slider(control_frame, "Right Velocity (mm/s)", 
                          self.right_velocity, -1000, 1000, 0)
        
        # Left Velocity Slider  
        self.create_slider(control_frame, "Left Velocity (mm/s)", 
                          self.left_velocity, -1000, 1000, 1)
        
        # Distance Slider
        self.create_slider(control_frame, "Distance (mm)", 
                          self.distance, 0, 2000, 2)
        
        # PID Control Section
        pid_title = tk.Label(control_frame, text="PID Control Parameters", 
                            font=("Arial", 12, "bold"), 
                            bg='#2b2b2b', fg='cyan')
        pid_title.pack(pady=(20,5))
        
        # Kp Row (Right and Left side by side)
        kp_frame = tk.Frame(control_frame, bg='#2b2b2b')
        kp_frame.pack(fill='x', pady=3)
        
        right_kp_frame = tk.Frame(kp_frame, bg='#2b2b2b')
        right_kp_frame.pack(side=tk.LEFT, fill='x', expand=True, padx=5)
        tk.Label(right_kp_frame, text="Right Kp:", bg='#2b2b2b', fg='#FF6B9D', 
                font=("Arial", 9), width=10, anchor='w').pack(side=tk.LEFT)
        tk.Label(right_kp_frame, text="Current:", bg='#2b2b2b', fg='gray', 
                font=("Arial", 8)).pack(side=tk.LEFT)
        tk.Label(right_kp_frame, textvariable=self.right_vel_kp, bg='#2b2b2b', 
                fg='yellow', font=("Arial", 8, "bold"), width=6).pack(side=tk.LEFT, padx=2)
        right_kp_entry = tk.Entry(right_kp_frame, width=8, font=("Arial", 8))
        right_kp_entry.pack(side=tk.LEFT, padx=2)
        tk.Button(right_kp_frame, text="Set", 
                 command=lambda: self.set_pid_value(self.right_vel_kp, right_kp_entry, 0.0, 10.0, "Right Kp"),
                 bg='#4CAF50', fg='white', font=("Arial", 7), width=4).pack(side=tk.LEFT, padx=1)
        
        left_kp_frame = tk.Frame(kp_frame, bg='#2b2b2b')
        left_kp_frame.pack(side=tk.RIGHT, fill='x', expand=True, padx=5)
        tk.Label(left_kp_frame, text="Left Kp:", bg='#2b2b2b', fg='#6BCB77', 
                font=("Arial", 9), width=10, anchor='w').pack(side=tk.LEFT)
        tk.Label(left_kp_frame, text="Current:", bg='#2b2b2b', fg='gray', 
                font=("Arial", 8)).pack(side=tk.LEFT)
        tk.Label(left_kp_frame, textvariable=self.left_vel_kp, bg='#2b2b2b', 
                fg='yellow', font=("Arial", 8, "bold"), width=6).pack(side=tk.LEFT, padx=2)
        left_kp_entry = tk.Entry(left_kp_frame, width=8, font=("Arial", 8))
        left_kp_entry.pack(side=tk.LEFT, padx=2)
        tk.Button(left_kp_frame, text="Set", 
                 command=lambda: self.set_pid_value(self.left_vel_kp, left_kp_entry, 0.0, 10.0, "Left Kp"),
                 bg='#4CAF50', fg='white', font=("Arial", 7), width=4).pack(side=tk.LEFT, padx=1)
        
        # Ki Row
        ki_frame = tk.Frame(control_frame, bg='#2b2b2b')
        ki_frame.pack(fill='x', pady=3)
        
        right_ki_frame = tk.Frame(ki_frame, bg='#2b2b2b')
        right_ki_frame.pack(side=tk.LEFT, fill='x', expand=True, padx=5)
        tk.Label(right_ki_frame, text="Right Ki:", bg='#2b2b2b', fg='#FF6B9D', 
                font=("Arial", 9), width=10, anchor='w').pack(side=tk.LEFT)
        tk.Label(right_ki_frame, text="Current:", bg='#2b2b2b', fg='gray', 
                font=("Arial", 8)).pack(side=tk.LEFT)
        tk.Label(right_ki_frame, textvariable=self.right_vel_ki, bg='#2b2b2b', 
                fg='yellow', font=("Arial", 8, "bold"), width=6).pack(side=tk.LEFT, padx=2)
        right_ki_entry = tk.Entry(right_ki_frame, width=8, font=("Arial", 8))
        right_ki_entry.pack(side=tk.LEFT, padx=2)
        tk.Button(right_ki_frame, text="Set", 
                 command=lambda: self.set_pid_value(self.right_vel_ki, right_ki_entry, 0.0, 5.0, "Right Ki"),
                 bg='#4CAF50', fg='white', font=("Arial", 7), width=4).pack(side=tk.LEFT, padx=1)
        
        left_ki_frame = tk.Frame(ki_frame, bg='#2b2b2b')
        left_ki_frame.pack(side=tk.RIGHT, fill='x', expand=True, padx=5)
        tk.Label(left_ki_frame, text="Left Ki:", bg='#2b2b2b', fg='#6BCB77', 
                font=("Arial", 9), width=10, anchor='w').pack(side=tk.LEFT)
        tk.Label(left_ki_frame, text="Current:", bg='#2b2b2b', fg='gray', 
                font=("Arial", 8)).pack(side=tk.LEFT)
        tk.Label(left_ki_frame, textvariable=self.left_vel_ki, bg='#2b2b2b', 
                fg='yellow', font=("Arial", 8, "bold"), width=6).pack(side=tk.LEFT, padx=2)
        left_ki_entry = tk.Entry(left_ki_frame, width=8, font=("Arial", 8))
        left_ki_entry.pack(side=tk.LEFT, padx=2)
        tk.Button(left_ki_frame, text="Set", 
                 command=lambda: self.set_pid_value(self.left_vel_ki, left_ki_entry, 0.0, 5.0, "Left Ki"),
                 bg='#4CAF50', fg='white', font=("Arial", 7), width=4).pack(side=tk.LEFT, padx=1)
        
        # Kd Row
        kd_frame = tk.Frame(control_frame, bg='#2b2b2b')
        kd_frame.pack(fill='x', pady=3)
        
        right_kd_frame = tk.Frame(kd_frame, bg='#2b2b2b')
        right_kd_frame.pack(side=tk.LEFT, fill='x', expand=True, padx=5)
        tk.Label(right_kd_frame, text="Right Kd:", bg='#2b2b2b', fg='#FF6B9D', 
                font=("Arial", 9), width=10, anchor='w').pack(side=tk.LEFT)
        tk.Label(right_kd_frame, text="Current:", bg='#2b2b2b', fg='gray', 
                font=("Arial", 8)).pack(side=tk.LEFT)
        tk.Label(right_kd_frame, textvariable=self.right_vel_kd, bg='#2b2b2b', 
                fg='yellow', font=("Arial", 8, "bold"), width=6).pack(side=tk.LEFT, padx=2)
        right_kd_entry = tk.Entry(right_kd_frame, width=8, font=("Arial", 8))
        right_kd_entry.pack(side=tk.LEFT, padx=2)
        tk.Button(right_kd_frame, text="Set", 
                 command=lambda: self.set_pid_value(self.right_vel_kd, right_kd_entry, 0.0, 2.0, "Right Kd"),
                 bg='#4CAF50', fg='white', font=("Arial", 7), width=4).pack(side=tk.LEFT, padx=1)
        
        left_kd_frame = tk.Frame(kd_frame, bg='#2b2b2b')
        left_kd_frame.pack(side=tk.RIGHT, fill='x', expand=True, padx=5)
        tk.Label(left_kd_frame, text="Left Kd:", bg='#2b2b2b', fg='#6BCB77', 
                font=("Arial", 9), width=10, anchor='w').pack(side=tk.LEFT)
        tk.Label(left_kd_frame, text="Current:", bg='#2b2b2b', fg='gray', 
                font=("Arial", 8)).pack(side=tk.LEFT)
        tk.Label(left_kd_frame, textvariable=self.left_vel_kd, bg='#2b2b2b', 
                fg='yellow', font=("Arial", 8, "bold"), width=6).pack(side=tk.LEFT, padx=2)
        left_kd_entry = tk.Entry(left_kd_frame, width=8, font=("Arial", 8))
        left_kd_entry.pack(side=tk.LEFT, padx=2)
        tk.Button(left_kd_frame, text="Set", 
                 command=lambda: self.set_pid_value(self.left_vel_kd, left_kd_entry, 0.0, 2.0, "Left Kd"),
                 bg='#4CAF50', fg='white', font=("Arial", 7), width=4).pack(side=tk.LEFT, padx=1)
        
        # Distance mode checkbox
        dist_mode_frame = tk.Frame(control_frame, bg='#2b2b2b')
        dist_mode_frame.pack(pady=10)
        dist_mode_check = tk.Checkbutton(dist_mode_frame, text="Enable Distance Mode", 
                                         variable=self.distance_mode,
                                         bg='#2b2b2b', fg='white',
                                         selectcolor='#2b2b2b')
        dist_mode_check.pack()
        
        # Steering PID and Buttons side-by-side
        steer_button_frame = tk.Frame(control_frame, bg='#2b2b2b')
        steer_button_frame.pack(fill='x', pady=(15,0))
        
        # Left side: Steering PID
        steer_left = tk.Frame(steer_button_frame, bg='#2b2b2b')
        steer_left.pack(side=tk.LEFT, fill='both', expand=True, padx=(0, 10))
        
        steer_pid_label = tk.Label(steer_left, text="Steering Control PID:", 
                                  font=("Arial", 10, "bold"), 
                                  bg='#2b2b2b', fg='orange')
        steer_pid_label.pack(anchor='w', pady=(0,5))
        
        self.create_pid_input(steer_left, "Steering Kp", self.steer_kp, 0.0, 10.0)
        self.create_pid_input(steer_left, "Steering Ki", self.steer_ki, 0.0, 5.0)
        self.create_pid_input(steer_left, "Steering Kd", self.steer_kd, 0.0, 2.0)
        
        # Right side: Buttons (horizontal)
        button_right = tk.Frame(steer_button_frame, bg='#2b2b2b')
        button_right.pack(side=tk.LEFT, padx=(10, 0), anchor='n', pady=(30, 0))
        
        # Send Data Button
        self.send_btn = tk.Button(button_right, text="Send Data", 
                                 command=self.send_data,
                                 bg='#2196F3', fg='white',
                                 font=("Arial", 10), 
                                 width=12, height=2)
        self.send_btn.pack(side=tk.LEFT, padx=5)
        
        # Emergency Stop Button
        self.emergency_btn = tk.Button(button_right, text="EMERGENCY\nSTOP", 
                                     command=self.emergency_stop,
                                     bg='#f44336', fg='white',
                                     font=("Arial", 10, "bold"), 
                                     width=12, height=2)
        self.emergency_btn.pack(side=tk.LEFT, padx=5)
        
        # Reset Button
        self.reset_btn = tk.Button(button_right, text="Reset Values", 
                                 command=self.reset_values,
                                 bg='#FF9800', fg='white',
                                 font=("Arial", 10), 
                                 width=12, height=2)
        self.reset_btn.pack(side=tk.LEFT, padx=5)
        
        # Status display
        self.create_status_display()
        
    def create_slider(self, parent, label, variable, min_val, max_val, row):
        frame = tk.Frame(parent, bg='#2b2b2b')
        frame.pack(fill='x', pady=5)
        
        # Label
        tk.Label(frame, text=label, bg='#2b2b2b', fg='white', 
                font=("Arial", 10)).pack(anchor='w')
        
        # Slider frame
        slider_frame = tk.Frame(frame, bg='#2b2b2b')
        slider_frame.pack(fill='x', pady=5)
        
        # Min label
        tk.Label(slider_frame, text=str(min_val), bg='#2b2b2b', 
                fg='gray').pack(side=tk.LEFT)
        
        # Slider
        slider = tk.Scale(slider_frame, from_=min_val, to=max_val, 
                         orient=tk.HORIZONTAL, variable=variable,
                         bg='#404040', fg='white', highlightbackground='#2b2b2b',
                         troughcolor='#606060', activebackground='#2196F3')
        slider.pack(side=tk.LEFT, fill='x', expand=True, padx=10)
        slider.bind("<ButtonRelease-1>", lambda e: self.on_slider_change())
        
        # Max label
        tk.Label(slider_frame, text=str(max_val), bg='#2b2b2b', 
                fg='gray').pack(side=tk.RIGHT)
        
        # Value input frame
        value_frame = tk.Frame(frame, bg='#2b2b2b')
        value_frame.pack(fill='x', pady=2)
        
        # Value display (current value)
        value_label = tk.Label(value_frame, text="Current:", bg='#2b2b2b', 
                              fg='white', font=("Arial", 9))
        value_label.pack(side=tk.LEFT)
        
        current_value_label = tk.Label(value_frame, textvariable=variable, bg='#2b2b2b', 
                                      fg='yellow', font=("Arial", 9, "bold"))
        current_value_label.pack(side=tk.LEFT, padx=5)
        
        # Manual input
        tk.Label(value_frame, text="Set:", bg='#2b2b2b', fg='white', 
                font=("Arial", 9)).pack(side=tk.LEFT, padx=(20,0))
        
        entry = tk.Entry(value_frame, width=8, font=("Arial", 9))
        entry.pack(side=tk.LEFT, padx=5)
        
        set_btn = tk.Button(value_frame, text="Set", 
                           command=lambda: self.set_value_from_entry(variable, entry, min_val, max_val),
                           bg='#4CAF50', fg='white', font=("Arial", 8),
                           width=6)
        set_btn.pack(side=tk.LEFT, padx=2)
        
    def create_pid_input(self, parent, label, variable, min_val, max_val):
        frame = tk.Frame(parent, bg='#2b2b2b')
        frame.pack(fill='x', pady=3)
        
        # Main container
        container = tk.Frame(frame, bg='#2b2b2b')
        container.pack(fill='x')
        
        # Label (fixed width for alignment)
        label_widget = tk.Label(container, text=label + ":", bg='#2b2b2b', fg='white', 
                               font=("Arial", 10), width=12, anchor='w')
        label_widget.pack(side=tk.LEFT, padx=(0,10))
        
        # Current value display
        tk.Label(container, text="Current:", bg='#2b2b2b', fg='gray', 
                font=("Arial", 9)).pack(side=tk.LEFT)
        
        current_value_label = tk.Label(container, textvariable=variable, bg='#2b2b2b', 
                                      fg='yellow', font=("Arial", 9, "bold"), width=8)
        current_value_label.pack(side=tk.LEFT, padx=(2,15))
        
        # Input field
        tk.Label(container, text="Set:", bg='#2b2b2b', fg='white', 
                font=("Arial", 9)).pack(side=tk.LEFT)
        
        entry = tk.Entry(container, width=10, font=("Arial", 9))
        entry.pack(side=tk.LEFT, padx=5)
        
        # Set button
        set_btn = tk.Button(container, text="Set", 
                           command=lambda: self.set_pid_value(variable, entry, min_val, max_val, label),
                           bg='#4CAF50', fg='white', font=("Arial", 8),
                           width=6)
        set_btn.pack(side=tk.LEFT, padx=2)
        
    def create_status_display(self):
        status_frame = tk.LabelFrame(self.root, text="Current Values", 
                                   bg='#2b2b2b', fg='white')
        status_frame.pack(pady=10, padx=20, fill='x')
        
        self.status_text = tk.Text(status_frame, height=6, width=60,
                                  bg='#404040', fg='white',
                                  font=("Consolas", 9))
        self.status_text.pack(pady=5)
        self.update_status_display()
        
    def update_status_display(self):
        self.status_text.delete(1.0, tk.END)
        status = f"Right Velocity: {self.right_velocity.get():.1f} mm/s\n"
        status += f"Left Velocity:  {self.left_velocity.get():.1f} mm/s\n"
        status += f"Distance:       {self.distance.get():.1f} mm {'(ACTIVE)' if self.distance_mode.get() else '(INACTIVE)'}\n"
        status += f"Right PID: Kp={self.right_vel_kp.get():.2f}, Ki={self.right_vel_ki.get():.2f}, Kd={self.right_vel_kd.get():.2f}\n"
        status += f"Left PID:  Kp={self.left_vel_kp.get():.2f}, Ki={self.left_vel_ki.get():.2f}, Kd={self.left_vel_kd.get():.2f}\n"
        status += f"Steer PID: Kp={self.steer_kp.get():.2f}, Ki={self.steer_ki.get():.2f}, Kd={self.steer_kd.get():.2f}\n"
        status += f"Connection:     {'Connected' if self.connected else 'Disconnected'}"
        self.status_text.insert(1.0, status)
        
        # Schedule next update
        self.root.after(100, self.update_status_display)
        
    def toggle_connection(self):
        if not self.connected:
            self.esp_ip = self.ip_entry.get()
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.esp_ip, self.esp_port))
                self.connected = True
                self.connect_btn.config(text="Disconnect", bg='#f44336')
                self.status_label.config(text="Connected", fg='green')
            except Exception as e:
                messagebox.showerror("Connection Error", f"Failed to connect: {e}")
                self.connected = False
        else:
            self.connected = False
            try:
                self.client_socket.close()
            except:
                pass
            self.connect_btn.config(text="Connect", bg='#4CAF50')
            self.status_label.config(text="Disconnected", fg='red')

            
    def on_slider_change(self):
        if self.auto_send.get() and self.connected:
            self.send_data()
    
    def set_value_from_entry(self, variable, entry, min_val, max_val):
        try:
            value = float(entry.get())
            if min_val <= value <= max_val:
                variable.set(value)
                entry.delete(0, tk.END)  # Clear the entry box
                if self.auto_send.get() and self.connected:
                    self.send_data()
            else:
                messagebox.showwarning("Invalid Value", 
                                     f"Value must be between {min_val} and {max_val}")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid number")
            
    def set_pid_value(self, variable, entry, min_val, max_val, param_name):
        try:
            value = float(entry.get())
            if min_val <= value <= max_val:
                variable.set(value)
                entry.delete(0, tk.END)  # Clear the entry box
                if self.auto_send.get() and self.connected:
                    self.send_data()
            else:
                messagebox.showwarning("Invalid PID Value", 
                                     f"{param_name} must be between {min_val} and {max_val}")
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a valid number for PID parameter")
            
    def send_data(self):
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to ESP32 first!")
            return
            
        try:
            # Create data string with separate left/right motor PID: "right_vel,left_vel,distance,emergency,right_kp,right_ki,right_kd,left_kp,left_ki,left_kd,steer_kp,steer_ki,steer_kd,distance_mode"
            data = f"{self.right_velocity.get():.1f},{self.left_velocity.get():.1f},{self.distance.get():.1f},0,"
            data += f"{self.right_vel_kp.get():.3f},{self.right_vel_ki.get():.3f},{self.right_vel_kd.get():.3f},"
            data += f"{self.left_vel_kp.get():.3f},{self.left_vel_ki.get():.3f},{self.left_vel_kd.get():.3f},"
            data += f"{self.steer_kp.get():.3f},{self.steer_ki.get():.3f},{self.steer_kd.get():.3f},"
            data += f"{1 if self.distance_mode.get() else 0}\n"
            
            # Send data in separate thread to avoid GUI freezing
            threading.Thread(target=self._send_data_thread, args=(data,), daemon=True).start()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send data: {str(e)}")
            
    def _send_data_thread(self, data):
        try:
            self.client_socket.send(data.encode())
            response = self.client_socket.recv(1024).decode().strip()
            print(f"ESP32 Response: {response}")
            
            if ',' in response and self.cubemonitor_enabled.get():
                self.update_cubemonitor_data(response)
                
        except socket.timeout:
            self.root.after(0, lambda: messagebox.showerror("Timeout", "Connection to ESP32 timed out!"))
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Connection Error", f"Failed to send data: {str(e)}"))

            
    def emergency_stop(self):
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to ESP32 first!")
            return
            
        try:
            # Send emergency stop with current PID values
            data = f"0.0,0.0,0.0,1,"
            data += f"{self.right_vel_kp.get():.3f},{self.right_vel_ki.get():.3f},{self.right_vel_kd.get():.3f},"
            data += f"{self.left_vel_kp.get():.3f},{self.left_vel_ki.get():.3f},{self.left_vel_kd.get():.3f},"
            data += f"{self.steer_kp.get():.3f},{self.steer_ki.get():.3f},{self.steer_kd.get():.3f},0\n"
            threading.Thread(target=self._send_data_thread, args=(data,), daemon=True).start()
            
            # Reset sliders to zero
            self.right_velocity.set(0)
            self.left_velocity.set(0)
            self.distance.set(0)
            
            messagebox.showinfo("Emergency Stop", "Emergency stop sent!")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send emergency stop: {str(e)}")
            
    def reset_values(self):
        self.right_velocity.set(0)
        self.left_velocity.set(0)
        self.distance.set(0)
        # Reset PID to default values
        self.right_vel_kp.set(1.0)
        self.right_vel_ki.set(0.0)
        self.right_vel_kd.set(0.0)
        self.left_vel_kp.set(1.0)
        self.left_vel_ki.set(0.0)
        self.left_vel_kd.set(0.0)
        self.steer_kp.set(1.0)
        self.steer_ki.set(0.0)
        self.steer_kd.set(0.0)
    
    def toggle_cubemonitor(self):
        """Start/Stop CubeMonitor HTTP server"""
        if self.cubemonitor_enabled.get():
            self.start_cubemonitor_server()
        else:
            self.stop_cubemonitor_server()
    
    def start_cubemonitor_server(self):
        """Start HTTP server for STM32CubeMonitor"""
        try:
            # Create simple HTTP server that serves data in JSON format
            handler = self.create_cubemonitor_handler()
            self.cubemonitor_server = HTTPServer(('localhost', 8000), handler)
            
            # Start server in background thread
            server_thread = threading.Thread(target=self.cubemonitor_server.serve_forever, daemon=True)
            server_thread.start()
            
            self.cube_status_label.config(text="Server: ON (Port 8000)", fg='green')
            print("CubeMonitor server started on http://localhost:8000")
            print("Configure CubeMonitor to connect to: http://localhost:8000/data")
        except Exception as e:
            messagebox.showerror("Server Error", f"Failed to start CubeMonitor server: {e}")
            self.cubemonitor_enabled.set(False)
    
    def stop_cubemonitor_server(self):
        """Stop HTTP server"""
        if self.cubemonitor_server:
            self.cubemonitor_server.shutdown()
            self.cubemonitor_server = None
            self.cube_status_label.config(text="Server: OFF", fg='gray')
            print("CubeMonitor server stopped")
    
    def create_cubemonitor_handler(self):
        """Create HTTP request handler for CubeMonitor"""
        app_instance = self
        
        class CubeMonitorHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                pass  # Suppress console logs
            
            def do_GET(self):
                if self.path == '/data':
                    # Send data in JSON format that CubeMonitor can parse
                    data = app_instance.get_cubemonitor_data()
                    
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(json.dumps(data).encode())
                else:
                    self.send_response(404)
                    self.end_headers()
        
        return CubeMonitorHandler
    
    def get_cubemonitor_data(self):
        """Format data for STM32CubeMonitor"""
        # Return latest received data from Teensy
        return self.latest_data
    
    def update_cubemonitor_data(self, data_string):
        """Parse received data and update for CubeMonitor"""
        try:
            # Expected format from Teensy: "distance,orientation,left_vel,right_vel,servo_angle,cte"
            parts = data_string.strip().split(',')
            if len(parts) >= 6:
                self.latest_data = {
                    "timestamp": time.time(),
                    "robot_distance_mm": float(parts[0]),
                    "orientation_deg": float(parts[1]),
                    "left_velocity_mm_s": float(parts[2]),
                    "right_velocity_mm_s": float(parts[3]),
                    "servo_angle_deg": float(parts[4]),
                    "cross_track_error_mm": float(parts[5])
                }
        except Exception as e:
            print(f"Error updating CubeMonitor data: {e}")

def main():
    root = tk.Tk()
    app = RobotTuningInterface(root)
    root.mainloop()

if __name__ == "__main__":
    main()