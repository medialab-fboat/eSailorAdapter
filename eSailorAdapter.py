#!/usr/bin/env python3
import time
import rospy
from mavros_msgs.msg import OverrideRCIn, WaypointList, Waypoint, State, RCOut
from mavros_msgs.srv import *
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Quaternion
from geopy.distance import geodesic
from math import atan2, asin, degrees, sqrt, radians
def main():

    # esailor_adapter node start
    rospy.init_node('esailor_adapter', anonymous=True)

    actuatorsCommandTopicManager = ActuatorsCommandTopicManager()    
    pixhawkDataTopicManager = PixhawkDataTopicManager()

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        pixhawkDataTopicManager.publishExtractedData()
    
    rospy.spin()


class ActuatorsCommandTopicManager:    
    def __init__(self):        
        self.rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)    
        
        self.actuator_command_entrance_pub = rospy.Subscriber('/esailor_adapter/actuator_command_entrance', Int32MultiArray, self.command_entrance_callback)
        
        rospy.loginfo("Inicializado tópico /esailor_adapter/actuator_command_entrance")
    
    def command_entrance_callback(self, data):
        rospy.loginfo("Comandos recebidos: {}".format(data.data))

        rospy.loginfo("Value 1: {:.2f}, Value 2: {:.2f}, Value 3: {:.2f}".format(data.data[0], data.data[1], data.data[2]))
        self.update_pwm(data.data)


    def update_pwm(self, pwm_array):                
        pwm_msg = self.createMessageUpdateChannels(pwm_array)
        self.rc_pub.publish(pwm_msg)        
        #rospy.loginfo("Updated channels 1, 2 and 3 ")

    def createMessageUpdateChannels(self, pwm_array):
        rc_channels = [1500] * 18  # Inicialize all channels with (1500us)
        rc_channels[0] = pwm_array[0]  # Pixhawk channel(RCIn) 1
        rc_channels[1] = pwm_array[1]  # Pixhawk channel(RCIn) 2
        rc_channels[2] = pwm_array[2]  # Pixhawk channel(RCIn) 3

        msg = OverrideRCIn()
        msg.channels = rc_channels

        return msg


class PixhawkDataTopicManager:
    def __init__(self):        
        self.pixhawk_extracted_data_pub = rospy.Publisher('/esailor_adapter/pixhawk_extracted_data', Float64MultiArray, queue_size=10)
        self.extractedMessage = None

        #IMU
        self.orientation = None
        self.imu_topic = '/mavros/imu/data'
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        

        #GPS
        self.waypoint_sub = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoint_callback)
        self.current_position_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        self.waypoint_list = WaypointList()
        self.current_position = None

        self.dataArray = None

        #Bussola
        self.current_yaw = 0
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.handle_compass_hdg_rad)


        #Pixhawk Channels
        self.rcIn_topic = '/mavros/rc/out'
        rospy.Subscriber(self.rcIn_topic, RCOut, self.rc_callback)
        
        
        self.distanceToTarget = 0
        self.angleBetweenFowardAndTarget = 0
        self.surgeSpeed = 1.3
        self.apparentWindSpeed = 1.4
        self.apparentWindAngle = 1.5
        self.boomAngle = 1.6
        self.rudderAngle = 1.7
        self.electricPropulsionPower = 0
        self.rollAngle = 0


    def imu_callback(self, data):        
        self.orientation = data.orientation
        quaternion = Quaternion(self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w)

        # Convertion quaternion to Euler angle
        roll = atan2(2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x),
                    quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        
        #roll_degrees = roll * (180.0 / 3.14159)        
        self.rollAngle = degrees(roll)
        

    def waypoint_callback(self, waypoints):
        if waypoints.waypoints:
            self.waypoint_list = waypoints
            self.calculate_distance()          
            
    def position_callback(self, position):
        self.current_position = position
        self.calculate_distance()

    def rc_callback(self, data):
        #print("PWM of channel 1:", data.channels[0])        
        self.electricPropulsionPower = int(data.channels[0])

    def handle_compass_hdg_rad(self, data):
        self.current_yaw = data.data        

    def calculate_distance(self):        
        if self.waypoint_list.waypoints and self.current_position:
            rospy.wait_for_service('mavros/mission/pull')
            try:
                wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)                
                wpPullService().wp_received

            except rospy.ServiceException:
                print("Service Puling call failed:")
            
            #If there is at least one waypoint beside vehicle current position waypoint in pixhawk
            if len(self.waypoint_list.waypoints) > 1:
                #self.waypoint_list.waypoints[0] is the vehicle current position waypoint as mavros default
                #self.waypoint_list.waypoints[1] is the first waypoint in th pixhawk mission
                waypoint_lat = self.waypoint_list.waypoints[1].x_lat
                waypoint_lon = self.waypoint_list.waypoints[1].y_long
            
                vehicle_lat = self.current_position.latitude
                vehicle_lon = self.current_position.longitude

                #rospy.loginfo(f'Latitude waypoint{waypoint_lat}')
                #rospy.loginfo(f'Longitude waypoint{waypoint_lon}')
                #rospy.loginfo(f'Latitude veiculo{vehicle_lat}')
                #rospy.loginfo(f'Longitude veiculo{vehicle_lon}')

                self.distanceToTarget = self.calculate_distance_between_points(waypoint_lat, waypoint_lon, vehicle_lat, vehicle_lon)
                
                self.angleBetweenFowardAndTarget = self.calculate_direction_between_waypoints(waypoint_lat, waypoint_lon, vehicle_lat, vehicle_lon)                
                #print(f'*****Distance from vehicle to waypoint: {self.distanceToTarget} meters')
    
    def calculate_direction_between_waypoints(self, lat1, lon1, lat2, lon2):
        # waypoint_current and waypoint_destination are tuples containing coordinates (latitude, longitude)
        # vehicle_orientation is the angle in degrees to which the front of the vehicle is pointing

        # Calculate vectors representing the directions
        vector_current_to_destination = (
            lat1 - lat2,
            lon1 - lon2
        )

        # Calculate the angle between the vectors using the atan2 function
        angle_between_directions = atan2(vector_current_to_destination[1], vector_current_to_destination[0])

        # Convert the angle to degrees
        angle_between_directions_degrees = degrees(angle_between_directions)

        # Adjust the angle to ensure it is in the range [0, 360)
        angle_between_directions_degrees = (angle_between_directions_degrees + 360) % 360

        # Take into account the orientation of the vehicle
        final_direction = (angle_between_directions_degrees - self.current_yaw + 360) % 360

        return final_direction

    def calculate_distance_between_points(self, lat1, lon1, lat2, lon2):
        # Using geopy lib to calculate geodesic distance
        waypoint_coords = (lat1, lon1)
        vehicle_coords = (lat2, lon2)
        distance = geodesic(waypoint_coords, vehicle_coords).meters

        return distance
    

    def publishExtractedData(self):
        self.fillDataArray()
        self.pixhawk_extracted_data_pub.publish(self.dataArray)
            

    def fillDataArray(self):
        self.dataArray = Float64MultiArray(data=[self.distanceToTarget,
                                            self.angleBetweenFowardAndTarget,
                                            self.surgeSpeed, 
                                            self.apparentWindSpeed,
                                            self.apparentWindAngle,
                                            self.boomAngle,
                                            self.rudderAngle,
                                            self.electricPropulsionPower,
                                            self.rollAngle])


if __name__ == '__main__':    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    