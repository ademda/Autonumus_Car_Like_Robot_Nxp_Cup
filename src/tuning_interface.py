import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import time

class RobotTuningInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("NxP Cup Robot Tuning Interface")
        self.root.geometry("700x650")
        self.root.configure(bg='#2b2b2b')
        
        # Connection settings
        self.esp_ip = "192.168.4.1"
        self.esp_port = 8080
        self.connected = False
        
        # Current values
        self.right_velocity = tk.DoubleVar(value=0.0)
        self.left_velocity = tk.DoubleVar(value=0.0)
        self.distance = tk.DoubleVar(value=0.0)
        
        # PID parameters for velocity control
        self.vel_kp = tk.DoubleVar(value=1.0)
        self.vel_ki = tk.DoubleVar(value=0.0)
        self.vel_kd = tk.DoubleVar(value=0.0)
        
        # PID parameters for steering control
        self.steer_kp = tk.DoubleVar(value=1.0)
        self.steer_ki = tk.DoubleVar(value=0.0)
        self.steer_kd = tk.DoubleVar(value=0.0)
        
        # Distance mode control
        self.distance_mode = tk.BooleanVar(value=False)
        
        self.setup_ui()
        
    def setup_ui(self):
        # Title
        title_label = tk.Label(self.root, text="Robot Tuning Interface", 
                              font=("Arial", 14, "bold"), 
                              bg='#2b2b2b', fg='white')
        title_label.pack(pady=5)
        
        # Connection frame
        conn_frame = tk.Frame(self.root, bg='#2b2b2b')
        conn_frame.pack(pady=2)
        
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
        
        # Main control frame
        control_frame = tk.Frame(self.root, bg='#2b2b2b')
        control_frame.pack(pady=5, padx=10, fill='both', expand=True)
        
        # Right Velocity Slider
        self.create_slider(control_frame, "Right Velocity (mm/s)", 
                          self.right_velocity, -1000, 1000, 0)
        
        # Left Velocity Slider  
        self.create_slider(control_frame, "Left Velocity (mm/s)", 
                          self.left_velocity, -1000, 1000, 1)
        
        # Distance Slider
        self.create_slider(control_frame, "Distance (mm)", 
                          self.distance, 0, 2000, 2)
        
        # Distance Mode Checkbox
        distance_mode_frame = tk.Frame(control_frame, bg='#2b2b2b')
        distance_mode_frame.pack(fill='x', pady=2)
        
        self.distance_mode_check = tk.Checkbutton(distance_mode_frame, 
                                                 text="Enable Distance Mode", 
                                                 variable=self.distance_mode,
                                                 bg='#2b2b2b', fg='yellow',
                                                 selectcolor='#2b2b2b',
                                                 font=("Arial", 9),
                                                 command=self.on_distance_mode_change)
        self.distance_mode_check.pack(anchor='w', padx=5)
        
        # PID Control Section
        pid_title = tk.Label(control_frame, text="PID Control Parameters", 
                            font=("Arial", 11, "bold"), 
                            bg='#2b2b2b', fg='cyan')
        pid_title.pack(pady=(8,3))
        
        # Velocity PID
        vel_pid_label = tk.Label(control_frame, text="Velocity PID:", 
                                font=("Arial", 9, "bold"), 
                                bg='#2b2b2b', fg='lightgreen')
        vel_pid_label.pack(anchor='w', pady=(5,2))
        
        self.create_pid_input(control_frame, "Velocity Kp", self.vel_kp, 0.0, 10.0)
        self.create_pid_input(control_frame, "Velocity Ki", self.vel_ki, 0.0, 5.0)
        self.create_pid_input(control_frame, "Velocity Kd", self.vel_kd, 0.0, 2.0)
        
        # Steering PID
        steer_pid_label = tk.Label(control_frame, text="Steering PID:", 
                                  font=("Arial", 9, "bold"), 
                                  bg='#2b2b2b', fg='orange')
        steer_pid_label.pack(anchor='w', pady=(5,2))
        
        self.create_pid_input(control_frame, "Steering Kp", self.steer_kp, 0.0, 10.0)
        self.create_pid_input(control_frame, "Steering Ki", self.steer_ki, 0.0, 5.0)
        self.create_pid_input(control_frame, "Steering Kd", self.steer_kd, 0.0, 2.0)
        
        # Buttons frame
        button_frame = tk.Frame(self.root, bg='#2b2b2b')
        button_frame.pack(pady=5)
        
        # Send Data Button
        self.send_btn = tk.Button(button_frame, text="Send Data", 
                                 command=self.send_data,
                                 bg='#2196F3', fg='white',
                                 font=("Arial", 12), 
                                 width=12, height=2)
        self.send_btn.pack(side=tk.LEFT, padx=10)
        
        # Emergency Stop Button
        self.emergency_btn = tk.Button(button_frame, text="EMERGENCY\nSTOP", 
                                     command=self.emergency_stop,
                                     bg='#f44336', fg='white',
                                     font=("Arial", 12, "bold"), 
                                     width=12, height=2)
        self.emergency_btn.pack(side=tk.LEFT, padx=10)
        
        # Reset Button
        self.reset_btn = tk.Button(button_frame, text="Reset Values", 
                                 command=self.reset_values,
                                 bg='#FF9800', fg='white',
                                 font=("Arial", 12), 
                                 width=12, height=2)
        self.reset_btn.pack(side=tk.LEFT, padx=10)
        
        # Status display
        self.create_status_display()
        
        # Auto-send checkbox
        auto_frame = tk.Frame(self.root, bg='#2b2b2b')
        auto_frame.pack(pady=2)
        
        self.auto_send = tk.BooleanVar()
        auto_check = tk.Checkbutton(auto_frame, text="Auto-send on slider change", 
                                   variable=self.auto_send,
                                   bg='#2b2b2b', fg='white',
                                   selectcolor='#2b2b2b')
        auto_check.pack()
        
    def create_slider(self, parent, label, variable, min_val, max_val, row):
        frame = tk.Frame(parent, bg='#2b2b2b')
        frame.pack(fill='x', pady=2)
        
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
        frame.pack(fill='x', pady=1)
        
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
        status_frame.pack(pady=3, padx=10, fill='x')
        
        self.status_text = tk.Text(status_frame, height=4, width=50,
                                  bg='#404040', fg='white',
                                  font=("Consolas", 8))
        self.status_text.pack(pady=5)
        self.update_status_display()
        
    def update_status_display(self):
        self.status_text.delete(1.0, tk.END)
        status = f"Right Velocity: {self.right_velocity.get():.1f} mm/s\n"
        status += f"Left Velocity:  {self.left_velocity.get():.1f} mm/s\n"
        status += f"Distance:       {self.distance.get():.1f} mm {'(ACTIVE)' if self.distance_mode.get() else '(INACTIVE)'}\n"
        status += f"Vel PID: Kp={self.vel_kp.get():.2f}, Ki={self.vel_ki.get():.2f}, Kd={self.vel_kd.get():.2f}\n"
        status += f"Steer PID: Kp={self.steer_kp.get():.2f}, Ki={self.steer_ki.get():.2f}, Kd={self.steer_kd.get():.2f}\n"
        status += f"Connection:     {'Connected' if self.connected else 'Disconnected'}"
        self.status_text.insert(1.0, status)
        
        # Schedule next update
        self.root.after(100, self.update_status_display)
        
    def toggle_connection(self):
        if not self.connected:
            self.esp_ip = self.ip_entry.get()
            self.connected = True
            self.connect_btn.config(text="Disconnect", bg='#f44336')
            self.status_label.config(text="Connected", fg='green')
        else:
            self.connected = False
            self.connect_btn.config(text="Connect", bg='#4CAF50')
            self.status_label.config(text="Disconnected", fg='red')
            
    def on_slider_change(self):
        if self.auto_send.get() and self.connected:
            self.send_data()
    
    def on_distance_mode_change(self):
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
            # Create data string: "right_vel,left_vel,distance,emergency,vel_kp,vel_ki,vel_kd,steer_kp,steer_ki,steer_kd,distance_mode"
            data = f"{self.right_velocity.get():.1f},{self.left_velocity.get():.1f},{self.distance.get():.1f},0,"
            data += f"{self.vel_kp.get():.3f},{self.vel_ki.get():.3f},{self.vel_kd.get():.3f},"
            data += f"{self.steer_kp.get():.3f},{self.steer_ki.get():.3f},{self.steer_kd.get():.3f},"
            data += f"{1 if self.distance_mode.get() else 0}\n"
            
            # Send data in separate thread to avoid GUI freezing
            threading.Thread(target=self._send_data_thread, args=(data,), daemon=True).start()
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send data: {str(e)}")
            
    def _send_data_thread(self, data):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(3)  # 3 second timeout
                s.connect((self.esp_ip, self.esp_port))
                s.send(data.encode())
                response = s.recv(1024)
                print(f"ESP32 Response: {response.decode().strip()}")
                
        except socket.timeout:
            self.root.after(0, lambda: messagebox.showerror("Timeout", "Connection to ESP32 timed out!"))
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Connection Error", f"Failed to send data: {str(e)}"))
            
    def emergency_stop(self):
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to ESP32 first!")
            return
            
        try:
            # Send emergency stop: all zeros with emergency flag, keep current PID values
            data = f"0.0,0.0,0.0,1,{self.vel_kp.get():.3f},{self.vel_ki.get():.3f},{self.vel_kd.get():.3f},"
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
        self.vel_kp.set(1.0)
        self.vel_ki.set(0.0)
        self.vel_kd.set(0.0)
        self.steer_kp.set(1.0)
        self.steer_ki.set(0.0)
        self.steer_kd.set(0.0)

def main():
    root = tk.Tk()
    app = RobotTuningInterface(root)
    root.mainloop()

if __name__ == "__main__":
    main()