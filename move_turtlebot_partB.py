import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class TurtleBotControlNode(Node):
    def __init__(self):
        super().__init__('turtlebot_control_node')
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Movement State
        self.vel_msg = Twist()
        self.safety_threshold = 0.3
        self.obstacle_detected = False
        
        # Speed step increments
        self.LIN_STEP = 0.05
        self.ANG_STEP = 0.1
        
        # Terminal settings for keyboard input
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Create a timer to run the keyboard loop
        self.create_timer(0.1, self.run_loop)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def scan_callback(self, msg):
        # Check a 20-degree arc in front (0-10 and 350-359)
        front_indices = list(range(0, 11)) + list(range(350, 360))
        
        self.obstacle_detected = False
        for i in front_indices:
            # Check if index exists in current scan
            if i < len(msg.ranges):
                distance = msg.ranges[i]
                if msg.range_min < distance < self.safety_threshold:
                    self.obstacle_detected = True
                    break

    def run_loop(self):
        key = self.get_key()

        # Keyboard Mapping
        if key == 'w':
            self.vel_msg.linear.x += self.LIN_STEP
        elif key == 's':
            self.vel_msg.linear.x -= self.LIN_STEP
        elif key == 'a':
            self.vel_msg.angular.z += self.ANG_STEP
        elif key == 'd':
            self.vel_msg.angular.z -= self.ANG_STEP
        elif key == ' ': # Emergency Stop
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
        elif key == '\x03': # Ctrl+C
            rclpy.shutdown()

        # SAFETY OVERRIDE
        # If moving forward and obstacle detected, force stop
        if self.obstacle_detected and self.vel_msg.linear.x > 0:
            self.get_logger().warn("OBSTACLE DETECTED! Stopping forward motion.")
            self.vel_msg.linear.x = 0.0

        self.cmd_pub.publish(self.vel_msg)

def main(args=None):
    print('Turtlebot3 Move Part B')
    print('Controls: W/S (Linear), A/D (Angular), Space (Stop), Ctrl+C (Exit)')
    
    rclpy.init(args=args)
    node = TurtleBotControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Final safety stop
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
