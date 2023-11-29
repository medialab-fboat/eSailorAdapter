#!/usr/bin/env python3
import time
import rospy
from mavros_msgs.msg import OverrideRCIn, WaypointList, Waypoint
from mavros_msgs.srv import *
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Quaternion
from geopy.distance import geodesic
from math import atan2, asin

def main():

    # Inicializa o nó ROS com o nome "leitor_comandos"
    rospy.init_node('esailor_adapter', anonymous=True)

    actuatorsCommandTopicManager = ActuatorsCommandTopicManager()    
    pixhawkDataTopicManager = PixhawkDataTopicManager()


    rate = rospy.Rate(3)  # 10 Hz

    while not rospy.is_shutdown():
        pixhawkDataTopicManager.publishExtractedData()
    
    rospy.spin()
    
    
    '''
    rospy.loginfo("Antes do While " )
    
    try:
        while not rospy.is_shutdown():    
            channel = 1
            pwm_value = 1500

            rospy.loginfo("dentro do While " )
            actuatorsCommandTopicManager.update_pwm(channel)
            
            if channel >= 6:
                channel = 1
            else:
                channel = channel + 1

            time.sleep(0.5)

    except rospy.ROSInterruptException:
        pass
    '''

class ActuatorsCommandTopicManager:    
    def __init__(self):
        # Inicializa o nó ROS com o nome "leitor_comandos"
        #rospy.init_node('mavros_node_2', anonymous=True, log_level=rospy.DEBUG) 
        
        self.rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)    
        
        self.actuator_command_entrance_pub = rospy.Subscriber('/esailor_adapter/actuator_command_entrance', Int32MultiArray, self.command_entrance_callback)
        
        rospy.loginfo("Inicializado tópico /esailor_adapter/actuator_command_entrance")
    
    def command_entrance_callback(self, data):    
        # Aqui, você pode processar os dados conforme necessário
        rospy.loginfo("Comandos recebidos: {}".format(data.data))

        # Adicione a impressão dos valores recebidos
        rospy.loginfo("Valor 1: {:.2f}, Valor 2: {:.2f}, Valor 3: {:.2f}".format(data.data[0], data.data[1], data.data[2]))
        self.update_pwm(data.data)


    def update_pwm(self, pwm_array):        
        
        rospy.loginfo("dentro do update " )
               
        pwm_msg = self.createMessageUpdateChannels(pwm_array)

        rospy.loginfo("Antes de alterar canais 1, 2 e 3 ")

        self.rc_pub.publish(pwm_msg)
        
        rospy.loginfo("Alterados o canais 1, 2 e 3 ")

    def createMessageUpdateChannels(self, pwm_array):
        rospy.loginfo("dentro do createMessageUpdateChannels " )
        rc_channels = [1500] * 18  # Inicializa todos os canais com neutro (1500us)
        rc_channels[0] = pwm_array[0]  # Define o valor PWM desejado no canal 1
        rc_channels[1] = pwm_array[1]  # Define o valor PWM desejado no canal 2
        rc_channels[2] = pwm_array[2]  # Define o valor PWM desejado no canal 3

        msg = OverrideRCIn()
        msg.channels = rc_channels

        return msg


class PixhawkDataTopicManager:
    def __init__(self):
        'Distance to target in meters'
        self.distToTarget = 0
        self.pixhawk_extracted_data_pub = rospy.Publisher('/esailor_adapter/pixhawk_extracted_data', Float64MultiArray, queue_size=10)
        self.extractedMessage = None


        #IMU
        self.imu_topic = '/mavros/imu/data'        
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)

        #GPS
        self.waypoint_sub = rospy.Subscriber('/mavros/mission/waypoints', WaypointList, self.waypoint_callback)
        self.current_position_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        self.waypoint_list = WaypointList()
        self.current_position = None


        self.dataArray = None
        
        self.distanceToTarget = 0
        self.angleBetweenFowardAndTarget = 1.2
        self.surgeSpeed = 1.3
        self.apparentWindSpeed = 1.4
        self.apparentWindAngle = 1.5
        self.boomAngle = 1.6
        self.rudderAngle = 1.7
        self.electricPropulsionPower = 1.8
        self.rollAngle = 0


    def imu_callback(self, data):        
        orientation = data.orientation
        quaternion = Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

        # Convertion quaternion to Euler angle
        roll = atan2(2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x),
                    quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        
        roll_degrees = roll * (180.0 / 3.14159)        
        self.rollAngle = roll_degrees
        print("***Inclinação do Roll: {:.2f} graus".format(roll_degrees))


    def waypoint_callback(self, waypoints):
        if waypoints.waypoints:
            self.waypoint_list = waypoints
            self.calculate_distance()          
            
    def position_callback(self, position):
        self.current_position = position
        self.calculate_distance()

    def calculate_distance(self):
        #self.quantityOfWayPointsInPixhawk = 0
        if self.waypoint_list.waypoints and self.current_position:
            rospy.wait_for_service('mavros/mission/pull')
            try:
                wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)
                #self.quantityOfWayPointsInPixhawk = wpPullService().wp_received
                wpPullService().wp_received

                #print("self.quantityOfWayPointsInPixhawk")
                #print(self.quantityOfWayPointsInPixhawk)
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

                rospy.loginfo(f'Latitude waypoint{waypoint_lat}')
                rospy.loginfo(f'Longitude waypoint{waypoint_lon}')
                rospy.loginfo(f'Latitude veiculo{vehicle_lat}')
                rospy.loginfo(f'Longitude veiculo{vehicle_lon}')

                self.distanceToTarget = self.calculate_distance_between_points(waypoint_lat, waypoint_lon, vehicle_lat, vehicle_lon)
                print(f'*****Distance from vehicle to waypoint: {self.distanceToTarget} meters')
            #else:
             #   print('There is not any waypoint setted in pixhawk current mission')

    def calculate_distance_between_points(self, lat1, lon1, lat2, lon2):
        # Utilizando a biblioteca geopy para calcular a distância geodésica
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



'''
class NodeManager:
    def __init__(self, tipo, velocidade):
        self.tipo = tipo
        self.velocidade = velocidade
    
    def toString(self):
        print(f'Este carro é um {self.tipo} e está andando a {self.velocidade} quilômetros por hora')





class PixhawkDataAdapter:
    def __init__(self, tipo, velocidade):
        self.tipo = tipo
        self.velocidade = velocidade
    
    def toString(self):
        print(f'Este carro é um {self.tipo} e está andando a {self.velocidade} quilômetros por hora')

'''           

if __name__ == '__main__':    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    