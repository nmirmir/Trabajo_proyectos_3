#!/usr/bin/env python2
# -*- coding: utf-8 -*-

'''
Hacer launch que ejecute un código que calibre la cámara y luego se llame al código principal

Código Launch:
    1. Llama al código que calibra la cámara
    2. publica la información en un 
    2. Llama al código que ejecuta el código principal

Código principal:
    Pasos de ejecución de funciones:
        1.  Vamos a preprocesar el frame
        2.  Detectamos los códigos aruco
        3.  Calculammos una homografía utilizando los 4 arucos del medio, cogeremos las 4 esquinas de cada aruco
            A estos les asignaremos una id en específico
        4. Calculamos una homografía y cambiamos el frame en crudo por el frame al que se le ha aplicado la homografía
        5. Asignar una ROI a cada almacén
        6. Detectamos los arucos que hay en cada uno de los almacenes y los almacenamos en un diccionario de forma que Nº almacén : Lata_1 ...
        7. Leemos el topic del robot: 
            7.1 Acción a realizar:
                7.1.1 Mover lata: 
                    7.1.1.1 Topic
                        7.1.1.1.1 Información que se recibe [Topic --> info_robot_mover_lata]
                            -lata que tenemos
                            -almacén origen // en teoría lo sé ya
                            -almacén destino
                        7.1.1.1.1.2 Información que se envía [Topic --> info_camara_mover_lata]
                            -Acción a realizar
                        7.1.1.1.2 Información que se recibe [Topic --> info_robot_mover_a_lata]
                            -lata que tenemos
                            -almacén origen // en teoría lo sé ya
                            -almacén destino
                        7.1.1.1.2.1 Información que se envía [Topic --> info_camara_mover_a_lata]
                            -Acción a realizar
                    7.1.1.2 Actuamos con la información recibida del topic
                        7.1.1.2.1 Llamamos a la función de cálculo de distancia entre arucos diciéndole:
                            - Qué lata llevamos
                            - Cuál es nuestro origen
                            - Cuál es el destino objetivo
                        7.1.1.2.2 Llamamos a la función en la que vamos a aplicar un algoritmo A*que permite que el robot navegue:
                            // Cada acción hará referencia a un estado y formarán un grafo
                                ·- Estados:
                                    -Recto
                                    -Parar
                                    -Atras
                                    -GirarD
                                    -GirarI
                                    -Destino
                            // Cada vez que se llegue a un estado:
                            7.1.1.2.2.1 Publicamos en [Topic --> info_camara_mover_lata]
                                - La orden que se asocia con los estados

                        
                7.1.2 Moverse a lata: Topic --> info_mover_a_lata
                    7.1.1.1 Topic
                        7.1.1.1.1 Información que se recibe [Topic --> info_robot_mover_a_lata]
                            -almacén origen // en teoría lo sé ya
                            -almacén destino                                
                        7.1.1.1.2 Información que se envía [Topic --> info_camara_mover_a_lata]
                    
            
                    





'''



import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool

def order_points(pts):
    """
    Ordena 4 puntos (x,y) en el orden:
    top-left, top-right, bottom-right, bottom-left
    """
    rect = np.zeros((4,2), dtype="float32")
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).ravel()
    rect[0] = pts[np.argmin(s)]       # esquina superior izquierda
    rect[2] = pts[np.argmax(s)]       # esquina inferior derecha
    rect[1] = pts[np.argmin(diff)]    # esquina superior derecha
    rect[3] = pts[np.argmax(diff)]    # esquina inferior izquierda
    return rect

class EvitarRobot:
    def __init__(self):
        rospy.init_node("aruco_detector", anonymous=True)

        # Parámetros del sistema
        self.frame_size               = (640, 480)
        self.threshold_distance_r1_r2 = 150  # umbral entre robots ID 1-5 y 6-10
        self.align_threshold          = 30
        self.threshold_distance       = 20

        # Coordenadas del mundo virtual (para vista cenital)
        self.world_pts = np.array([
            [ 60,  60],
            [300,  60],
            [300, 180],
            [ 60, 180],
        ], dtype=np.float32)

        # Configuración del detector ArUco
        self.aruco_dict  = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters_create()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
        params.cornerRefinementMethod  = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_params = params

        # Estado interno
        self.centre_pts_img = []  # almacena hasta 4 centros para homografía
        self.H = None  # matriz de homografía

        # Subscripciones y publicaciones ROS
        self.sub   = rospy.Subscriber("/csi_cam_0/image_raw", Image, self.cb, queue_size=1)
        self.alert_pub  = rospy.Publisher("/alerta_robot",     Int32, queue_size=1)
        self.pickup_pub = rospy.Publisher("/go_pickup",        Bool,  queue_size=1)

        # Timers periódicos para lógica auxiliar
        rospy.Timer(rospy.Duration(0.2), self._on_distance_timer)
        rospy.Timer(rospy.Duration(0.2), self._on_reset_timer)

        # Ventanas de visualización
        cv2.namedWindow("bird_view", cv2.WINDOW_NORMAL)
        cv2.namedWindow("live",      cv2.WINDOW_NORMAL)

    def cb(self, msg):
        # Convierte imagen ROS a array OpenCV y procesa
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = arr.reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.resize(frame, self.frame_size)
        self.process_frame(frame)

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1) Detección de marcadores ArUco
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params
        )
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for c, i in zip(corners, ids.flatten()):
                if 11 <= i <= 50:
                    ctr = tuple(np.mean(c.reshape(4,2), axis=0).astype(int))
                    if ctr not in self.centre_pts_img:
                        self.centre_pts_img.append(ctr)

        # 2) Cálculo de homografía si hay 4 puntos válidos
        if self.H is None and len(self.centre_pts_img) >= 4:
            pts = np.array(self.centre_pts_img[:4], dtype=np.float32)
            pts_ord = order_points(pts)
            H, mask = cv2.findHomography(pts_ord, self.world_pts, cv2.RANSAC, 5.0)
            if H is not None:
                self.H = H
                rospy.loginfo("Homografía establecida con puntos: %s", pts_ord)
            else:
                rospy.logwarn("findHomography ha fallado")

        # 3) Generación de vista cenital (bird view)
        bird = frame.copy()
        if self.H is not None:
            bird = cv2.warpPerspective(frame, self.H, (360,240))
        cv2.imshow("bird_view", bird)

        # 4) Cálculo de distancia entre robots (IDs 1-5 y 6-10)
        if ids is not None:
            def last_center(range_ids):
                pts = [c for c, i in zip(corners, ids.flatten()) if range_ids[0] <= i <= range_ids[1]]
                if not pts: return None
                return tuple(np.mean(pts[-1].reshape(4,2),axis=0).astype(int))

            p1 = last_center((1,5))
            p2 = last_center((6,10))
            if p1 and p2:
                cv2.line(frame, p1, p2, (0,255,255), 2)
                d = np.linalg.norm(np.array(p1)-np.array(p2))
                cv2.putText(frame, "d=%.1f"%d,
                            ((p1[0]+p2[0])//2, (p1[1]+p2[1])//2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),2)
                cv2.circle(frame, p1, int(self.threshold_distance_r1_r2), (0,0,255),2)
                flag = 1 if d < self.threshold_distance_r1_r2 else 0
                print("[DIST] %.1f < %d -> alerta=%d"%(d, self.threshold_distance_r1_r2, flag))
                self.alert_pub.publish(Int32(flag))

        cv2.imshow("live", frame)
        cv2.waitKey(1)

    def _on_distance_timer(self, evt):
        # Placeholder para tareas periódicas
        pass

    def _on_reset_timer(self, evt):
        # Si hay más de 4 centros, conservar sólo los primeros 4 para homografía
        if self.H is None and len(self.centre_pts_img) > 4:
            self.centre_pts_img = self.centre_pts_img[:4]

if __name__ == "__main__":
    EvitarRobot()
    rospy.spin()
