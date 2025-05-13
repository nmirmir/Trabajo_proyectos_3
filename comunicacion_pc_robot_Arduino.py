#!/usr/bin/env python3

# IMPORTANTE ESTE CODIGO ES EN EL ROBOT
#sUBSCRIBIRSE AL TOPIC DEL TRADUCTOR PARA ABIR Y CERRAR PINZA

import rospy
import serial
from std_msgs.msg import String

# Configuración del puerto serie (ajustar según el puerto de tu sistema)
ser = serial.Serial('COM3', 9600)  # Usa el puerto y la velocidad de baudios correctos (ej. COM3 para Windows, /dev/ttyUSB0 para Linux)

def callback(msg):
    """
    Función que será llamada cuando se reciba un mensaje del topic.
    Envía un mensaje a Arduino y luego publica una respuesta.
    """
    rospy.loginfo(f"Mensaje recibido: {msg.data}")
    msg_rec = msg.data
    # Enviar el mensaje al Arduino a través del puerto serie
    ser.write(msg_rec.encode())  # El mensaje debe ser convertido a bytes (utf-8)

    #rospy.loginfo(f"Mensaje enviado a Arduino: {msg.data}")
    
    # Publicar un mensaje en otro topic
    pub = rospy.Publisher('/accion_pinza', String, queue_size=10)

    pub.publish(msg_rec)

def listener():
    """
    Función principal que inicializa el nodo y el subscriber.
    """
    rospy.init_node('listenerArduino', anonymous=True)

    # Suscribirse al topic que espera recibir el mensaje
    rospy.Subscriber('/accion_pinza', String, callback)
    
    # Mantener el nodo activo
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()  # Llama a la función principal para iniciar el nodo
    except rospy.ROSInterruptException:
        pass
    finally:
        # Cerrar el puerto serie al finalizar
        ser.close()
