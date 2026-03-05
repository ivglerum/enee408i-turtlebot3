import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

# Settings for non-blocking terminal read
def get_key():
    tty.setraw(sys.stdin.fileno())
    # Wait 0.1 seconds for a keypress, otherwise return None
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def move():
    rospy.init_node('turtlebot3_autonomous_move', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    vel_msg = Twist()

    # speed step increments
    LIN_STEP = 0.01
    ANG_STEP = 0.1

    while not rospy.is_shutdown():
        key = get_key()
            
        if key == 'w':
            vel_msg.linear.x += LIN_STEP
        elif key == 's':
            vel_msg.linear.x -= LIN_STEP
        elif key == 'a':
            vel_msg.angular.z += ANG_STEP
        elif key == 'd':
            vel_msg.angular.z -= ANG_STEP
        elif key == ' ': # Spacebar to emergency stop
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        elif key == '\x03': # Ctrl+C
            break
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
