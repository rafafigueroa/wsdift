import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from turtlesim.msg import Pose

x = None
y = None
h = None
v = None
w = None


def twist_callback(tsim_Twist):
    global x, y, h, v, w

    v = tsim_Twist.linear.x
    w = tsim_Twist.angular.z


def start():
    """Simulates turtlebot movement
    and plots result continuously"""

    global x, y, h, v, w
    initial_time = True

    global pub
    pub = rospy.Publisher('turtle1/pose', Pose, queue_size=100)
    rospy.Subscriber("turtle1/cmd_vel", Twist, Twist_callback)
    # starts the node
    rospy.init_node('tsim')
    r = rospy.Rate(100)

    while not rospy.is_shutdown():

        now = rospy.get_rostime()
        tsim = now.secs
        if initial_time:
            previous_t = tsim
            delta_t = 0
            initialTime = False
        else:
            delta_t = tsim - previous_t
            previous_t = tsim
            t = t + delta_t

        r.sleep()


if __name__ == '__main__':
    start()

