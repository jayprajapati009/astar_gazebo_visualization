from math import atan2
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

import rospy

# defining origin
x = 0.0
y = 0.0
theta = 0.0


def newOdom(msg):
    """define the odom msg

    Args:
        msg (<class>): msg recieved from topic "\odom"
    """
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rotation = msg.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion(
        [rotation.x, rotation.y, rotation.z, rotation.w])


def runTut(path):
    """runs the turtlebot

    Args:
        path (lis): path to traverse
    """
    rospy.init_node("speed_controller")
    sub_vel = rospy.Subscriber("/odom", Odometry, newOdom)
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    speed = Twist()
    rate = rospy.Rate(4)
    goal = Point()

    for current_x, current_y, _ in path:
        print("Navigating to the goal point ....")
        goal.x = (current_x - 50)/100
        goal.y = -(current_y-100)/100
        while not rospy.is_shutdown():
            if abs(x - goal.x) < 0.1 and abs(y - goal.y) < 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                pub_vel.publish(speed)
                rate.sleep()
                break

            else:
                dx = goal.x - x
                dy = goal.y - y

                angle_to_achieve = atan2(dy, dx)

                if abs(angle_to_achieve - theta) > 0.1:

                    if angle_to_achieve - theta > 0:
                        speed.linear.x = 0.0
                        speed.angular.z = 0.1
                    elif angle_to_achieve - theta < 0:
                        speed.linear.x = 0.0
                        speed.angular.z = -0.1
                else:
                    speed.linear.x = 0.2
                    speed.angular.z = 0.0

                pub_vel.publish(speed)
                rate.sleep()
    print("\n")
    print("#" * 80)
    print("Reached the goal Node!")
    print("#" * 80)
