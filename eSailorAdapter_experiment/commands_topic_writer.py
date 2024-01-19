#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Int32MultiArray

def main():
    rospy.init_node('pwm_sender', anonymous=True)    
    command_pub = rospy.Publisher('/esailor_adapter/actuator_command_entrance', Int32MultiArray, queue_size=10)    
    rate = rospy.Rate(1)

    MINIMUM_PWM = 1000
    MAXMIUM_PWM = 2000
    executionCounter = 0

    a = 1300
    b = 1000
    c = 1200

    while not rospy.is_shutdown():        
        command_data = Int32MultiArray(data=[a, b, c])

        a = a + 5
        
        if executionCounter % 10 == 0:
            b = MINIMUM_PWM
        elif executionCounter % 5 == 0:
            b = MAXMIUM_PWM
        
        c = c + 9

        rospy.loginfo("before publish")        
        command_pub.publish(command_data)
        rospy.loginfo("after publish") 
        executionCounter += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
