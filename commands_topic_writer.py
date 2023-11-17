#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Int32MultiArray

def main():
    # Inicializa o nó ROS com o nome "veiculo"
    rospy.init_node('pwm_sender', anonymous=True)

    # Cria um objeto Publisher para publicar comandos no tópico "/comandos"
    command_pub = rospy.Publisher('/esailor_adapter/actuator_command_entrance', Int32MultiArray, queue_size=10)

    # Taxa de publicação (10 Hz neste exemplo)
    rate = rospy.Rate(1)

    a = 1300
    b = 1190
    c = 1200

    while not rospy.is_shutdown():
        # Cria um array de três posições com números float (substitua esses valores conforme necessário)
        command_data = Int32MultiArray(data=[a, b, c])

        a = a + 5;
        b = b + 7;
        c = c + 9;

        rospy.loginfo("antes de publicar")

        # Publica os comandos no tópico "/comandos"
        command_pub.publish(command_data)

        rospy.loginfo("depois de publicar")

        # Espera até a próxima iteração
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
