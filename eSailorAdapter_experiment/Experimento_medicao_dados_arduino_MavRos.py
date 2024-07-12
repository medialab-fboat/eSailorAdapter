import RPi.GPIO as GPIO
import time
from datetime import datetime


class FileWriter:
    file_name = ""

    def __init__(self):
        self.file_name = "measurement/esailorAdapter_data_travel_time" + datetime.now().strftime("%d_%m_%Y %H_%M_%S_%f")[:-3]

    def writeInFile(self, msg):
        try:
            with open(self.file_name +'.txt', 'a') as file:
                file.write(msg + ";" + datetime.now().strftime("%H:%M:%S:%f")[:-3] + "\n")
        except KeyboardInterrupt:
            print("Fail when tried write file.")
            
#Instancia FileWriter            
fileWriter = FileWriter()

# Configura o modo de numeração dos pinos (BOARD é o mais intuitivo)
GPIO.setmode(GPIO.BOARD)

# Lista de pinos a serem usados
pins = [29, 31, 33, 35, 37]

# Configura os pinos como saída
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

def set_pins(value):
    # Converte o valor para uma string binária de 5 bits
    binary_value = f'{value:05b}'
    
    print(binary_value)
    
    # Define o estado dos pinos conforme a string binária
    for pin, bit in zip(pins, binary_value):
        GPIO.output(pin, GPIO.HIGH if bit == '1' else GPIO.LOW)
        fileWriter.writeInFile(f"Sent RUDDER_ANGLE parameter data to Arduino: {value}")

try:
    while True:
        for i in range(31):  # Itera de 0 até 60
            print(i)
            set_pins(i)
            time.sleep(0.3)  # Aguarda 1 segundo
            

except KeyboardInterrupt:
    # Limpa a configuração dos pinos quando o programa é interrompido
    GPIO.cleanup()
