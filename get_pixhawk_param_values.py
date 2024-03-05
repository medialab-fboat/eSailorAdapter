#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import Param

def param_callback(data):
    #rospy.loginfo("data -> value: %f", data.value)
        
    if data.param_id == "RUDDER_ANGLE":
        rospy.loginfo("Parameter 'RUDDER_ANGLE' value: "+ str(data.value.real))
    
    if data.param_id == "RUDDER_MOTOR":
        rospy.loginfo("Parameter 'RUDDER_MOTOR' value: "+ str(data.value.real))

    if data.param_id == "RUDDER_CURRENT":
        rospy.loginfo("Parameter 'RUDDER_CURRENT' value: "+ str(data.value.real))

    if data.param_id == "SAIL_CURRENT":
        rospy.loginfo("Parameter 'SAIL_CURRENT' value: "+ str(data.value.real))

    if data.param_id == "SAIL_MOTOR":
        rospy.loginfo("Parameter 'SAIL_MOTOR' value: "+ str(data.value.real))

    if data.param_id == "SAIL_ANGLE":
        rospy.loginfo("Parameter 'SAIL_ANGLE' value: "+ str(data.value.real))
        

def parameter_listener():
    rospy.init_node('parameter_listener', anonymous=True)

    param_sub = rospy.Subscriber('/mavros/param/param_value', Param, param_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        parameter_listener()
    except rospy.ROSInterruptException:
        pass
