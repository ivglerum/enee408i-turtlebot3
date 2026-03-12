import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

settings = termios.tcgetattr(sys.stdin)

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def save_image(msg):
    bridge = CvBridge()
    # Convert ROS Image message to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite("camera_image.jpg", cv_image)
    rospy.loginfo("Image saved as camera_image.jpg!")

def move():
    rospy.init_node('turtlebot3_teleop_cam', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    vel_msg = Twist()

    LIN_STEP = 0.05  # Increased slightly for better response
    ANG_STEP = 0.1

    rospy.loginfo("Controls: W/S (Linear), A/D (Angular), Space (Stop), Q (Snap), Ctrl+C (Exit)")

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
        elif key == ' ':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        elif key == 'q':
            rospy.loginfo("Capturing image...")
            try:
                # Wait for one frame from the camera topic (timeout 2s)
                msg = rospy.wait_for_message("/usb_cam/image_raw", Image, timeout=2.0)
                save_image(msg)
            except rospy.ROSException:
                rospy.logerr("Camera timeout: Is the usb_cam node running?")
        elif key == '\x03':
            break
            
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
