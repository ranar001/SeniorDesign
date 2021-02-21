#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Float64
import rospy
from sensor_msgs.msg import JointState

def Unity():
    rospy.init_node('UnitytoROS', anonymous=True)
    # setup joy topic subscription
    joy_subscriber = rospy.Subscriber("/unity_states", JointState, main, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def main(data):
    controller_1_publisher = rospy.Publisher('/phantomx_reactor_controller/shoulder_yaw_joint/command', Float64, queue_size = 10)
    controller_2_publisher = rospy.Publisher('/phantomx_reactor_controller/shoulder_pitch_joint/command', Float64, queue_size = 10)
    controller_3_publisher = rospy.Publisher('/phantomx_reactor_controller/elbow_pitch_joint/command', Float64, queue_size = 10)
    controller_4_publisher = rospy.Publisher('/phantomx_reactor_controller/wrist_pitch_joint/command', Float64, queue_size = 10)
    controller_5_publisher = rospy.Publisher('/phantomx_reactor_controller/wrist_roll_joint/command', Float64, queue_size = 10)


    # Create the topic message

    # Joint names for UR5


    rate = rospy.Rate(50) # 50hz

    rospy.loginfo(data)
    rospy.loginfo(data)

    # Set the points to the trajectory


    # Publish the message
    controller_1_publisher.publish(data.position[0])
    controller_2_publisher.publish(data.position[1])
    controller_3_publisher.publish(data.position[2])
    controller_4_publisher.publish(data.position[3])
    controller_5_publisher.publish(data.position[4])
    rate.sleep()


if __name__ == '__main__':
    try:
        Unity()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
