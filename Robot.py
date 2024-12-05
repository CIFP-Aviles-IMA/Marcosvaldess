#Aqui poneis el Docdtring que querais
"""
Desarrollo de un sistema de control para un brazo robótico, inicialmente programado en C++ y adaptado a Python. Este proyecto se basa en los tutoriales disponibles en YouTube indicados debajo.
https://www.youtube.com/watch?v=AIsVlgopqJc&t=794s y https://www.youtube.com/watch?v=OiQKw0lZ5Rw&t=18s. 
El primer paso consiste en traducir el código original de C++ a Python, ya que se utilizará una placa Jetson Nano para su implementación. 
Los pasos a seguir son los siguientes: 
-Importar las librerías necesarias. 
-Definir las variables que serán usadas. 
-Asignar los pines de entrada correspondientes a la Jetson Nano. 
-Configurar la inicialización y asociar los motores a los pines de entrada. 
-Conectar los motores al potenciómetro. Configurar la apertura y el cierre de la pinza. 
-Finalmente, se realiza una revisión del código para asegurar que no existan errores.
"""
#import wire 
#import Adafruit_PWMServodriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
import time
i2c = busio.I2C(broad.SCL, board.SDA)
from adafruit_servokit import Servokit

#Declaro variables globales
MIN_PULSE_WIDTH=    650
MAX_PULSE_WIDTH=    2350
FREQUENCY      =    50

#Instancio el Driver del controlador de servos
pwm = adafruit_pca9685.PCA9685("i2C")
kit = Servokit(channels=16)
potWrist    =   GPIO.input(15)
potElbow    =   GPIO.input(19)                              
potShoulder =   GPIO.input(21)            
portBase    =   GPIO.input(23)

#Configuro el SetUP
time.sleep(5)                       #<--- So I have time to get controller to starting position 
"pca.frequency" = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = "adafruit_motot".servo.Servo(1)  #cualquiera de los dos
wrist = "adafruit_motor".servo.Servo(1)
elbow = "adafruit_motor".servo.Servo(2)
shoulder = "adafruit_motor".servo.Servo(3) 
base = "adafruit_motor".servo.Servo(4)
potWrist = "adafruit_motor".servo.Servo(5)
potElbow = "adafruit_motor".servo.Servo(6)
potShoulder = "adafruit_motor".servo.Servo(7)
potBase = "adafruit_motor".servo.Servo(8)

pwm.begin()
pwm.setPWMFrenq(FREQUENCY)
pwm.setPWM(32, 0,90)                 #Set  Gripper to 90 degrees (Close Gripper ) x en Jetson x=32
GPIO.setup(13, GPIO.IN)           # Channel tiene que ser un pin valido para jetson

def moveMotor(controlIn, motorOut ):
    """
    A continuación vamos a describir las funciones que estan relacionadas con la función de MoveMotor(controlIN, motorOUT):

    Args:
    ControlIN (int): El  valor del potenciometro se lee en el pin GPIO correspondiente,el cual esta determinado por el pin seleccionado.
    MotorOUT (int): El pin de salida del motor se utiliza para enviar la señal PWM al motor que se va a controlar

    Returns:
    Su función principal es devolver el robot a la posición que indique el valor del potenciometro """
    
    pulse_wide, pulse_widht, potVal = -7

 #   portVal= analogRead(controlIn);
    PORTvAL= GPIO.input(controlIn)
    pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096); 

    pwm = GPIO.PWM(motorOut, 0, pulse_widht)

while (True): 
    moveMotor(potWrist, wrist)
    moveMotor(potElbow, elbow)
    moveMotor("portShoulder", shoulder)
    moveMotor(portBase, base)
    pushButton = GPIO.input(13)
    if("pushBotton" == GPIO.LOW):

     pwm.setPWM(hand, 0, 10)
    print("Grab")
else:   

        pwm.setPWM(hand)
        print("Release")
    
GPIO.cleanup()
setup()


#Asignamos pins
potWrist = A3
potElbow = A2                   #Assign Potentiometers to pins on Arduino Uno             
potShoulder = A1                
portBase = A0

hand =11
wrist =12
elbow =13                      #Assign Motors to pins on Servo Driver Board
shoulder =14
base =15