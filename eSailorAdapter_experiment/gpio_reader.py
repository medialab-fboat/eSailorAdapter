#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
from datetime import datetime


pwm_pin = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(pwm_pin, GPIO.IN)

def record_reading():
    reading = ""    
    file_name = "medicao/medicao_saida_pwm" + datetime.now().strftime("%d_%m_%Y %H_%M_%S_%f")[:-3]
    control_pulse_value = 0.0
    control_pulse_value_inicialized = False

    try:
        with open(file_name+'.txt', 'a') as arquivo:
            while True:               
                
                pulse_start = time.time()
                while GPIO.input(pwm_pin) == GPIO.LOW:
                    pulse_start = time.time()

                pulse_end = time.time()
                while GPIO.input(pwm_pin) == GPIO.HIGH:
                    pulse_end = time.time()

                pulse_duration = pulse_end - pulse_start

                agora = datetime.now()
                tempo_formatado = agora.strftime("%H:%M:%S:%f")[:-3]

                #transform
                pulse_duration = int((pulse_duration * 1e6))
                reading = "{};{}".format(pulse_duration, tempo_formatado)+"\n"
                           
                print(reading)
                arquivo.write(reading)

                time.sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Closing the program.")


if __name__ == "__main__":
    record_reading()