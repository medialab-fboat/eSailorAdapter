#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from math import atan2, asin

def imu_callback(data):
    # Callback chamada quando novos dados IMU são recebidos
    orientation = data.orientation
    quaternion = Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

    # Converte quaternion para ângulos de Euler
    roll = atan2(2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x),
                 quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z)

    # Converte a inclinação do roll de radianos para graus
    roll_degrees = roll * (180.0 / 3.14159)

    # Imprime a inclinação do roll
    print("Inclinação do Roll: {:.2f} graus".format(roll_degrees))

def get_roll_inclination():
    # Nome do tópico para receber dados IMU
    imu_topic = '/mavros/imu/data'

    # Inicializa o nó ROS
    rospy.init_node('get_roll_inclination', anonymous=True)

    # Define a função de callback para o tópico IMU
    rospy.Subscriber(imu_topic, Imu, imu_callback)

    # Aguarda até que o nó seja encerrado
    rospy.spin()

if __name__ == '__main__':
    try:
        get_roll_inclination()
    except rospy.ROSInterruptException:
        pass
