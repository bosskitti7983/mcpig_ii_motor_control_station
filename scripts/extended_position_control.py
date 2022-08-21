#!/usr/bin/python3

from sys import exit
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from math import pi

from time import sleep


# max_rpm = 70.0
# limit_speed_rpm = lambda input_percent : min(max( (float(input_percent)/max_rpm*100.0 )/60.0*2*pi , -max_rpm/60.0*2*pi), max_rpm/60.0*2*pi)   
# limit_speed_rpm = lambda input_percent : min(max( (float(input_percent)/max_rpm*100.0 )/60.0*2*pi , 0), max_rpm/60.0*2*pi)   
# speed =  1/.229  ==> 41rpm / 2/3 rps 

# speed = lambda input : 70/60*2*pi    

# speed = min(max(speed, -limit_speed) , limit_speed)

approx_wheel_radius = pi * 1e-3 * (39.2)

callback_name = None
callback_position = None
callback_velocity = None
callback_effort = None


def callback(data):
    global callback_name, callback_position, callback_velocity, callback_effort
    callback_name = data.name 
    callback_position = data.position
    callback_velocity = data.velocity
    callback_effort = data.effort

    # rospy.loginfo('get {}, {}, {}'. format(callback_position, callback_velocity, callback_effort))

def talker():
    motors_num = 1

    
    pub = rospy.Publisher('/db_dynamixel_ROS_driver/hexapod_state_commands', JointState, queue_size=10)

    # sub = rospy.Subscriber("/db_dynamixel_ROS_driver/dynamixel_state_list/dynamixel_state[0]/present_position", String, callback)
    sub = rospy.Subscriber("/db_dynamixel_ROS_driver/hexapod_joint_feedback", JointState, callback)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz

    joint_state = JointState()
    joint_state.name = ["id_1"] 
    joint_state.position = [0]

    joint_state.velocity = [0.1]
    joint_state.effort = [200]

    for i in range(motors_num):
        joint_state.velocity[i] = 0.1
        joint_state.effort[i] = 200

    current_distance = 0

    while callback_position is None:
        rospy.loginfo('wait for first position data')
        sleep(1)
    
    first_position = callback_position
    rospy.loginfo('wait for first position data {}'.format(first_position))

    moving_distance = 0

    while not rospy.is_shutdown():
   

        # input_rad_p_s = limit_speed_rpm(float(input('input rpm(max is 70)')))
        
        distance_input = input('unit m :')

        if distance_input != "":
            moving_distance = float(distance_input) 
        else :
            pass

        # try : 
        #     moving_distance = float() 
        # except Exception:
        #     exit('')

        # input_rad_p_s = limit_speed_rpm(float(rospy.get_param('/mcpig_ii_motor_torque_control/distance')))
        # moving_distance =  min(2.5, float( rospy.get_param('/mcpig_ii_motor_torque_control/speed') ))


        joint_state.velocity[0] = 0
        joint_state.position[0] = ((moving_distance) / approx_wheel_radius  * 2*pi )+ float(first_position[0])



        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub.publish(joint_state)
        rate.sleep()




if __name__ == '__main__':
    try:

        talker()
    
    except rospy.ROSInterruptException:
        exit('ros interrupt trigger')
