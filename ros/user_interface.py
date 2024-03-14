import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import font
from threading import Thread

class SensorGUI:
    def __init__(self, node, text_size=12):
        self.node = node
        self.window = tk.Tk()
        self.window.title("ROS Sensor and Motor Interface")

        # Set the font size for the labels
        label_font = font.Font(size=text_size)

        # Create labels for each type of sensor data
        self.pressure_label = tk.Label(self.window, text="Pressure: Waiting...", font=label_font)
        self.pressure_label.pack()

        self.tension_label = tk.Label(self.window, text="Tension: Waiting...", font=label_font)
        self.tension_label.pack()

        self.current_label = tk.Label(self.window, text="Current: Waiting...", font=label_font)
        self.current_label.pack()

        self.t_ext_label = tk.Label(self.window, text="External Temp: Waiting...", font=label_font)
        self.t_ext_label.pack()

        self.t_bat_label = tk.Label(self.window, text="Battery Temp: Waiting...", font=label_font)
        self.t_bat_label.pack()

        self.t_esc_label = tk.Label(self.window, text="ESC Temp: Waiting...", font=label_font)
        self.t_esc_label.pack()

        self.leak_label = tk.Label(self.window, text="Leak: Waiting...", font=label_font)
        self.leak_label.pack()

        self.quaternion_label = tk.Label(self.window, text="Quaternion: Waiting...", font=label_font)
        self.quaternion_label.pack()

        self.d_sonar_label = tk.Label(self.window, text="Sonar Distance: Waiting...", font=label_font)
        self.d_sonar_label.pack()

        self.motor_label = tk.Label(self.window, text="Motor Command Data: Waiting...", font=label_font)
        self.motor_label.pack()

        self.window.after(100, self.check_ros_callbacks)

    def check_ros_callbacks(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        self.window.after(100, self.check_ros_callbacks)

    def update_sensor_data(self, data):
        if len(data) == 12:
            self.pressure_label.config(text=f"Pressure (atm): {data[0]:.3f}")
            self.tension_label.config(text=f"Tension (V): {data[1]:.2f}")
            self.current_label.config(text=f"Current (mA): {data[2]:.2f}")
            self.t_ext_label.config(text=f"External Temp (°C): {data[3]:.1f}")
            self.t_bat_label.config(text=f"Battery Temp (°C): {data[4]:.1f}")
            self.t_esc_label.config(text=f"ESC Temp (°C): {data[5]:.1f}")
            self.leak_label.config(text=f"Leak (0-1): {data[6]:.3f}")
            quaternion = f"({data[7]:.2f}, {data[8]:.2f}, {data[9]:.2f}, {data[10]:.2f})"
            self.quaternion_label.config(text=f"Quaternion: {quaternion}")
            self.d_sonar_label.config(text=f"Sonar Distance (m): {data[11]:.3f}")

    def update_motor_data(self, data):
        # Assuming data is a string representation of the Twist message
        # Example format: "Linear: <Vector3>, Angular: <Vector3>"
        # Split the data into linear and angular components
        parts = data.split(", Angular: ")
        if len(parts) == 2:
            linear_data = parts[0].replace("Linear: ", "")
            angular_data = parts[1]
            formatted_data = f"Motor Commands\nLinear: {linear_data}\nAngular: {angular_data}"
            self.motor_label.config(text=formatted_data)
        else:
            self.motor_label.config(text="Motor Command Data: Invalid format")

    def run(self):
        self.window.mainloop()

class UserInterface(Node):
    def __init__(self, gui):
        super().__init__('user_interface')
        self.gui = gui
        self.sensor_subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_values',
            self.sensor_callback,
            10)
        self.motor_subscription = self.create_subscription(
            Twist,
            'motor_commands',
            self.motor_callback,
            10)

    def sensor_callback(self, msg):
        self.gui.update_sensor_data(msg.data)

    def motor_callback(self, msg):
        motor_data = f"Linear: {msg.linear}, Angular: {msg.angular}"
        self.gui.update_motor_data(motor_data)

def run_ros_node(node):
    rclpy.spin(node)
    node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface(None)

    gui = SensorGUI(node, text_size=22)  # Adjust the text size here
    node.gui = gui

    # Run the ROS node in a separate thread
    ros_thread = Thread(target=run_ros_node, args=(node,), daemon=True)
    ros_thread.start()

    # Run the Tkinter GUI in the main thread
    gui.run()
    
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
