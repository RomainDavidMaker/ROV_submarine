import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
from std_msgs.msg import Float32MultiArray

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('control_joystick')
        self.publisher_ = self.create_publisher(Twist, 'command', 10)
        self.light_publisher = self.create_publisher(Float32MultiArray, 'control_light', 10)


        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # Light toggle state and button press tracking
        self.light_state = [0.0, 0.0]  #  two lights, initially off
        self.last_x_pressed = False
        self.last_square_pressed = False

    def read_joystick(self):
        pygame.event.pump()
        twist = Twist()

        # Mapping joystick axes to Twist message fields

        """
        Left Stick Horizontal Axis: get_axis(0)
        Moves between -1 (left) and 1 (right).

        Left Stick Vertical Axis: get_axis(1)
        Moves between -1 (up) and 1 (down).

        Right Stick Horizontal Axis: get_axis(3)
        Moves between -1 (left) and 1 (right).

        Right Stick Vertical Axis: get_axis(4)
        Moves between -1 (up) and 1 (down).

        L2 Trigger: get_axis(2)
        Varies from -1 (not pressed) to 1 (fully pressed).
    
        R2 Trigger: get_axis(5)
        Varies from -1 (not pressed) to 1 (fully pressed).

        attention to the orientation of the axis :

        MOTOR MAPPING
            ^
            x
        <--y (z up)

        M1           M2
            M5  M6
            M8  M7
        M4           M3
        """

        twist.linear.x = -self.joystick.get_axis(1)
        twist.linear.y = -self.joystick.get_axis(0)
        twist.linear.z = (self.joystick.get_axis(2)+1)/2.0 - (self.joystick.get_axis(5)+1)/2.0
        twist.angular.x = self.joystick.get_axis(3)
        twist.angular.y = -self.joystick.get_axis(4)
        twist.angular.z = float(self.joystick.get_button(4)- self.joystick.get_button(5))
        self.publisher_.publish(twist)


        # Light toggle logic for X button
        x_pressed = self.joystick.get_button(0)  # Assuming button 0 is X
        if x_pressed and not self.last_x_pressed:
            self.light_state[0] = 1.0 if self.light_state[0] == 0.0 else 0.0  # Toggle first light
            self.last_x_pressed = True
        elif not x_pressed:
            self.last_x_pressed = False

        # Light toggle logic for Square button
        square_pressed = self.joystick.get_button(3)  # Assuming button 3 is Square
        if square_pressed and not self.last_square_pressed:
            self.light_state[1] = 1.0 if self.light_state[1] == 0.0 else 0.0  # Toggle second light
            self.last_square_pressed = True
        elif not square_pressed:
            self.last_square_pressed = False

        # Publish light state if either button is pressed
        if x_pressed or square_pressed:
            light_msg = Float32MultiArray(data=self.light_state)
            self.light_publisher.publish(light_msg)

        

def main(args=None):
    rclpy.init(args=args)
    joystick_control_node = JoystickControlNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(joystick_control_node, timeout_sec=0.1)
            joystick_control_node.read_joystick()
    except KeyboardInterrupt:
        pass
    finally:
        joystick_control_node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
