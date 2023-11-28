#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic

class WaypointDistanceCalculator:
    def __init__(self):
        rospy.init_node('waypoint_distance_calculator', anonymous=True)
        self.waypoint_sub = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoint_callback)
        self.current_position_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        self.waypoint_list = WaypointList()
        self.current_position = None
       
        # Define o waypoint (S 22° 57' 6.26", W 43° 12' 37.66")
        '''
        self.waypoint = Waypoint()
        self.waypoint.x_lat = -22.9123
        self.waypoint.y_long = -43.2265
        self.waypoint.z_alt = 0  # Altitude em metros

        self.waypoint_list.waypoints.append(self.waypoint)
        '''
    def waypoint_callback(self, waypoints):
        if waypoints.waypoints:
            self.waypoint_list = waypoints
            self.calculate_distance()          
            
    def position_callback(self, position):
        self.current_position = position
        self.calculate_distance()

    def calculate_distance(self):
        self.quantityOfWayPointsInPixhawk = 0
        if self.waypoint_list.waypoints and self.current_position:
            rospy.wait_for_service('mavros/mission/pull')
            try:
                wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)
                self.quantityOfWayPointsInPixhawk = wpPullService().wp_received

                print("self.quantityOfWayPointsInPixhawk")
                print(self.quantityOfWayPointsInPixhawk)
            except rospy.ServiceException:
                print("Service Puling call failed:")
            
            #If there is at least one waypoint beside vehicle current position waypoint in pixhawk
            if self.quantityOfWayPointsInPixhawk > 1:
                #self.waypoint_list.waypoints[0] is the vehicle current position waypoint as mavros default
                #self.waypoint_list.waypoints[1] is the first waypoint in th pixhawk mission
                waypoint_lat = self.waypoint_list.waypoints[1].x_lat
                waypoint_lon = self.waypoint_list.waypoints[1].y_long
            
                vehicle_lat = self.current_position.latitude
                vehicle_lon = self.current_position.longitude

                rospy.loginfo(f'Latitude waypoint{waypoint_lat}')
                rospy.loginfo(f'Longitude waypoint{waypoint_lon}')
                rospy.loginfo(f'Latitude veiculo{vehicle_lat}')
                rospy.loginfo(f'Longitude veiculo{vehicle_lon}')

                distance = self.calculate_distance_between_points(waypoint_lat, waypoint_lon, vehicle_lat, vehicle_lon)
                print(f'Distance from vehicle to waypoint: {distance} metros')
            #else:
             #   print('There is not any waypoint setted in pixhawk current mission')

    def calculate_distance_between_points(self, lat1, lon1, lat2, lon2):
        # Utilizando a biblioteca geopy para calcular a distância geodésica
        waypoint_coords = (lat1, lon1)
        vehicle_coords = (lat2, lon2)
        distance = geodesic(waypoint_coords, vehicle_coords).meters

        return distance

if __name__ == '__main__':
    try:
        waypoint_distance_calculator = WaypointDistanceCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
