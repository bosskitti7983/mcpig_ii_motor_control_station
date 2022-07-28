#!/usr/bin/python3

from sys import exit
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState



speed =  70/60 * 0.9
speed = min(max(speed, -7.15) , 7.15)

def talker():
    motors_num = 1

    pub = rospy.Publisher('/db_dynamixel_ROS_driver/hexapod_state_commands', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz

    joint_state = JointState()
    joint_state.name = ["id_1"] 
    joint_state.position = [0]

    joint_state.velocity = [0]
    joint_state.effort = [200]

    for i in range(motors_num):
        joint_state.velocity[i] = 0
        joint_state.effort[i] = 200
    # o1 = 0.9
    # o2 = 0.1

    while not rospy.is_shutdown():

        

        joint_state.velocity[0] = speed
        # joint_state.position[0] = 0.5

        hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub.publish(joint_state)
        rate.sleep()


if __name__ == '__main__':
    try:

        talker()
    
    except rospy.ROSInterruptException:
        exit('ros interrupt trigger')