#!/usr/bin/env python3

import subprocess
import re
import time

import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

#import random
#from playsound import playsound
#import pygame

#Crear variable global
pos_lata = None
pos_alm1 = None
pos_alm2 = None
pos_alcanzada = False
estado_pinza = "Abierto"


#Comando que nos lleva a la torre de inicio con la pinza abierta
#pygame.mixer.init()

def callback_lata(msg):
    global estado_pinza 
    estado_pinza = msg.data


def a_coger_lata():
    print("pedro porro: (Estoy pillando la lata)")
    global estado_pinza
    
    #1: enviarle al robot un mensaje con el estado de la pinza 
    pub = rospy.Publisher('/accion_pinza', String, queue_size=10)
    mensaje = "Cerrado"
    pub.publish(mensaje)

    #2: Creamos subscriber
    subscriber = rospy.Subscriber("/accion_pinza", String, callback_lata)

    #2: Esperar a variable global para recibir respuesta.
    while (estado_pinza == "Abierto"):
        time.sleep(1.0)
    subscriber.unregister()
    subscriber.shutdown()

    #C fini

    #if random.randint(1, 100) <= 35:
    #    print("BOEEEEEEE")
        #pygame.mixer.music.load("BOEEEE.mp3")
        #pygame.mixer.music.play()
        #while pygame.mixer.music.get_busy():
        #    pygame.time.Clock().tick(3)

def a_dejar_lata():
    print("pedro porro: (Estoy dejando la lata)")
    print("pedro porro: (Estoy pillando la lata)")
    global estado_pinza
    

    
    #1: enviarle al robot un mensaje con el estado de la pinza
    pub = rospy.Publisher('/accion_pinza', String, queue_size=10)
    mensaje = "Abierto"
    pub.publish(mensaje)

    #2: Creamos subscriber
    subscriber = rospy.Subscriber("/accion_pinza", String, callback_lata)

    #2: Esperar a variable global para recibir respuesta.
    while (estado_pinza == "Abierto"):
        time.sleep(1.0)

    subscriber.unregister()
    subscriber.shutdown()
    #C fini

    #Abrir Pinza
    #Pasar orden por serie al arduino

    #if random.randint(1, 100) <= 35:
    #    print("BOEEEEEEE")
        #pygame.mixer.music.load("BOEEEE.mp3")
        #pygame.mixer.music.play()
        #while pygame.mixer.music.get_busy():
        #    pygame.time.Clock().tick(3)


        

def mov(x,z):
    # Crea un publicador al topic cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Espera a que el publicador esté listo
    rospy.sleep(1)  # Espera un momento para que se conecte con los suscriptores

    # Crea el mensaje Twist
    vel_msg = Twist()
    vel_msg.linear.x = x
    vel_msg.angular.z = z

    # Publica el mensaje
    pub.publish(vel_msg)

    rospy.loginfo(f'Publicado: linear_x={linear_x}, angular_z={angular_z}')


 
def callback_mover_a_lata(msg):
    global pos_alcanzada
    rospy.loginfo(f"Mensaje recibido: {msg.data}")
    #Nos toca movernos a la posicoon de lata
    if msg.data == "Recto": mov(0.2,0.0)
    elif msg.data == "Parar": mov(0.0,0.0)
    elif msg.data == "Atras": mov(-0.2,0.0)
    elif msg.data == "GirarD": mov(0.0,0.2)
    elif msg.data == "GirarI": mov(0.0,-0.2)
    elif msg.data == "Destino" : pos_alcanzada = True



def a_mover_lata(lata, alm1,alm2):
    print("pedro porro: (Estoy moviendo la lata)")

    global pos_lata 
    global pos_alm1 
    global pos_alm2 
    global pos_alcanzada
   

    pos_lata = None
    pos_alm1 = None
    pos_alm2 = None

    '''
    1: enviarle a la camara a traves de un nodo la pieza y el almacen.
    mensaje 
    pub = rospy.Publisher('/info_robot_mover_lata', String, queue_size=10)
    mensaje = f'{{"lata": "{lata}", "alm1": "{alm1}", "alm2": "{alm2}"}}'
    pub.publish(mensaje)

    2: Creamos subscriber
    subscriber = rospy.Subscriber("/info_camara_mover_lata", String, callback_mover_a_lata)

    2: Esperar a variable global para recibir respuesta.
    while (pos_alcanzada == False):
        time.sleep(1.0)

    subscriber.unregister()
    subscriber.shutdown()
    pos_alcanzada = False


    3:Destino alcanzado
    '''
    #Pedirle posicion de la lata y el almacen destino


    #if random.randint(1, 100) <= 35:
    #    print("BOEEEEEEE")
        #pygame.mixer.music.load("BOEEEE.mp3")
        #pygame.mixer.music.play()
        #while pygame.mixer.music.get_busy():
        #    pygame.time.Clock().tick(3)

def a_mover_a_lata(lata, alm1):
    print("pedro porro: (Estoy moviendome hacia la lata)")
    
    global pos_lata 
    global pos_alm1 
    global pos_alm2
    global pos_alcanzada 
   

    pos_lata = None
    pos_alm1 = None
    pos_alm2 = None

    
    #1: enviarle a la camara a traves de un nodo la pieza y el almacen.
    mensaje 
    pub = rospy.Publisher('/info_robot_mover_a_lata', String, queue_size=10)
    mensaje = f'{{"lata": "{lata}", "alm1": "{alm1}"}}'
    pub.publish(mensaje)

    #2: Creamos subscriber
    subscriber = rospy.Subscriber("/info_camara_mover_a_lata", String, callback_mover_a_lata)

    #2: Esperar a variable global para recibir respuesta.
    while (pos_alcanzada == False):
        time.sleep(1.0)

    subscriber.unregister()
    subscriber.shutdown()
    pos_alcanzada = False

    #3:Destino alcanzado
    #Pedirle posicion de la lata y el almacen destino


    #if random.randint(1, 100) <= 35:
    #    print("BOEEEEEEE")
        #pygame.mixer.music.load("BOEEEE.mp3")
        #pygame.mixer.music.play()
        #while pygame.mixer.music.get_busy():
        #    pygame.time.Clock().tick(3)



''' Definir los archivos de dominio y problema 
 Os recomiendo que esten en el misma carpeta 
 que este script para no tener que ir liandola 
 con rutas '''
 º
domain_file = "pedro_damain_perf.pddl"
problem_file = "pedro_porro_perf.pddl"

# Comando para ejecutar Fast Downward
command = [
    "./fast-downward.py", 
    domain_file, 
    problem_file, 
    "--search", "astar(lmcut())"
]

# Ejecutar el comando y capturar la salida
print("funciona?")
try:
    result = subprocess.run(
        command, 
        stdout=subprocess.PIPE, 
        stderr=subprocess.PIPE, 
        universal_newlines=True, 
        check=True,
    )
    print("funciona!")
    print("Plan generado exitosamente.")
    # Procesar la salida para extraer las acciones como tuplas
    actions = []
    action_pattern = re.compile(r"(a_[^\)]*)")

    for match in action_pattern.findall(result.stdout):
        parts = match.strip().split()  
        action = tuple(parts)  
        actions.append(action)

    # Imprimir las acciones del plan que os dado PDDL
    print("Plan encontrado:")
    for act in actions:
        print(f"Accion : {act[0]}")

        if (act[0] == "a_coger_lata"):
            a_coger_lata()
        elif (act[0] == "a_dejar_lata"):
            a_dejar_lata()
        elif (act[0] == "a_mover_lata"):
            a_mover_lata(act[1], act[2] ,act[3])
        elif (act[0] == "a_mover_a_lata"):
            a_mover_a_lata(act[1], act[2])
    

    #en act se encuentran todas las acciones

    
    #Buscamos en el archivo de las acciones solo las lineas que empiezan por numero
    #para coger las acciones y quitar el restante


except subprocess.TimeoutExpired as e:
    print("⏰ Tiempo límite alcanzado. Se cancela la planificación.")
    # Opcional: manejar resultado parcial
    # e.stdout, e.stderr pueden tener texto útil
    print(f"Salida parcial:\n{e.stdout}")
    print(f"Error:\n{e.stderr}")

except subprocess.CalledProcessError as e:
    print(f"Error al ejecutar el Planner: {e}")
    print(f"Error: {e.stderr}")