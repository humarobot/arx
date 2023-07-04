#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose

def talker():
    pub = rospy.Publisher('target_tf', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        tf_msg = Pose()
        tf_msg.orientation.w = 80
        tf_msg.orientation.x = 0
        tf_msg.orientation.y = 1
        tf_msg.orientation.z = 0

        tf_msg.position.x = 0.1
        tf_msg.position.y = 0
        tf_msg.position.z = 0.2

        print("Message sent")
        pub.publish(tf_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass