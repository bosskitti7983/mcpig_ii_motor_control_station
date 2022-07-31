#!/usr/bin/python3

from sys import exit
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from math import pi

wheel_size_mm = 42
max_rpm = 70
limit_speed_rpm = lambda input_rpm : min(max( int(input_rpm)/60*2*pi , -max_rpm/60*2*pi), max_rpm/60*2*pi)   
# speed =  1/.229  ==> 41rpm / 2/3 rps 

speed = lambda input : 70/60*2*pi    

# speed = min(max(speed, -limit_speed) , limit_speed)

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
   
        input_rad_p_s = limit_speed_rpm(input('input rpm(max is 70)'))
        rospy.loginfo("set speed {rpm:.2f} rpm, (approx) {mmps:.2f} mm/s".format(rpm=input_rad_p_s*60/(2*pi), mmps=(input_rad_p_s/(2*pi)*pi*wheel_size_mm) ))


        joint_state.velocity[0] = input_rad_p_s
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