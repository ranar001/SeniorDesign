#!/usr/bin/python
#
# Send joint values from PhantoX to Gazebo using messages
#V3 inclue gripper control

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
from sensor_msgs.msg import JointState



def Phantom():
    rospy.init_node('Real_joint', anonymous=True)
    # setup joy topic subscription
    joy_subscriber = rospy.Subscriber("pris_states", JointState, main, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def main(data):

    pub = rospy.Publisher('/gripper_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    arm = JointTrajectory()
    arm.header = Header()
    # Joint names for UR5
    arm.joint_names = ['arm_gripper_right_joint','arm_gripper_left_joint']

    rate = rospy.Rate(200) # 50hz
    rospy.loginfo(arm)
    arm.header.stamp = rospy.Time.now()
    pts = JointTrajectoryPoint()
    pts.positions = [data.position[0],-data.position[0]]
    pts.time_from_start = rospy.Duration(1.0)
    # Set the points to the trajectory
    arm.points = []
    arm.points.append(pts)
    # Publish the message
    pub.publish(arm)
    rate.sleep()


if __name__ == '__main__':
    try:
        Phantom()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
