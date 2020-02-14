# !/usr/bin/env python


import rospy
from geometry_msgs.msg import Point


def talker():
    pub = rospy.Publisher('chatter', Point, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    msg = Point()
    msg.x = 1.0
    msg.y = 1.0
    msg.z = 0.0
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        msg.x += 0.0001
        msg.y += 0.0001
        msg.z += 0.0001
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


