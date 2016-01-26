#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def example_run():
    rospy.init_node('example')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        example_run()
    except rospy.ROSInterruptException:
        pass
