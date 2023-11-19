#!/usr/bin/env python3
import time
import rospy
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float64MultiArray


def main():

    # Inicializa o nó ROS com o nome "leitor_comandos"
    rospy.init_node('esailor_adapter', anonymous=True)

    actuatorsCommandTopicManager = ActuatorsCommandTopicManager()    
    pixhawkDataTopicManager = PixhawkDataTopicManager()


    rate = rospy.Rate(10)  # 10 Hz

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
        self.dataArray = None
        self.fillVariablesWithPixhawkValues()
        

    def publishExtractedData(self):
        if self.dataArray != None:
            self.fillVariablesWithPixhawkValues()
            self.pixhawk_extracted_data_pub.publish(self.dataArray)
        else:
            rospy.loginfo("There is not extracted data to publish on esailor_adapter/pixhawk_extracted_data topic")
            

    def fillVariablesWithPixhawkValues(self):
        self.distanceToTarget = self.getFromPixhawkDistanceToTarget()
        self.angleBetweenFowardAndTarget = self.getFromPixhawkAngleBetweenFowardAndTarget()
        self.surgeSpeed = self.getFromPixhawkSurgeSpeed()
        self.apparentWindSpeed = self.getFromPixhawkApparentWindSpeed()
        self.apparentWindAngle = self.getFromPixhawkApparentWindAngle()
        self.boomAngle = self.getFromPixhawkBoomAngle()
        self.rudderAngle = self.getFromPixhawkRudderAngle()
        self.electricPropulsionPower = self.getFromPixhawkElectricPropulsion()
        self.rollAngle = self.getFromPixhawkRollAngle()

        self.dataArray = Float64MultiArray(data=[self.distanceToTarget,
                                            self.angleBetweenFowardAndTarget,
                                            self.surgeSpeed, 
                                            self.apparentWindSpeed,
                                            self.apparentWindAngle,
                                            self.boomAngle,
                                            self.rudderAngle,
                                            self.electricPropulsionPower,
                                            self.rollAngle])


    
    def getFromPixhawkDistanceToTarget(self):
        return 1.1
    
    def getFromPixhawkAngleBetweenFowardAndTarget(self):
        return 1.2
    
    def getFromPixhawkSurgeSpeed(self):
        return 1.3
    
    def getFromPixhawkApparentWindSpeed(self):
        return 1.4
    
    def getFromPixhawkApparentWindAngle(self):
        return 1.5
    
    def getFromPixhawkBoomAngle(self):
        return 1.6
    
    def getFromPixhawkRudderAngle(self):
        return 1.7

    def getFromPixhawkElectricPropulsion(self):
        return 1.8
    
    def getFromPixhawkRollAngle(self):
        return 1.9



'''
class NodeManager:
    def __init__(self, tipo, velocidade):
        self.tipo = tipo
        self.velocidade = velocidade
    
    def toString(self):
        print(f'Este carro é um {self.tipo} e está andando a {self.velocidade} quilômetros por hora')


class PixhawkDataExtractor:
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
    