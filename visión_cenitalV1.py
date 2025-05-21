#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import os
import heapq
import json

'''


Los del robot tienen que guardar los estados, para que me sepan indicar qué es lo que quieren

Clasificación de ARUCOS:

    ZONA 1: ID = 1
        ARUCOS 9
    ZONA 2: ID = 2
        ARUCOS 10
    ZONA 3: ID = 3
        ARUCOS 11
    ZONA 4: ID = 4
        ARUCOS 12
    ZONA 5: ID = 5
        ARUCOS 13
    ZONA 6: ID = 6
        ARUCOS 14
    ZONA 7: ID = 7
    ZONA 8: ID = 8

    Código Launch:
    1. Llama al código que calibra la cámara
    2. publica la información en un 
    2. Llama al código que ejecuta el código principal

Código principal:
    Pasos de ejecución de funciones:
        1.  Vamos a preprocesar el frame --
        2.  Detectamos los códigos aruco --
            2.1 calculamos los centros de ellos, y los almacenamos en una lista
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
                        7.1.1.2.2 Llamamos a la función en la que vamos a aplicar un algoritmo A* que permite que el robot navegue:
                                ·- Estados:
                                    -Recto
                                    -Parar
                                    -Atras
                                    -GirarD
                                    -GirarI
                                    -Destino
                            7.1.1.2.2.1 Publicamos en [Topic --> info_camara_mover_lata]
                                - La orden que se asocia con los estados
                7.1.2 Moverse a lata: Topic --> info_mover_a_lata
                    7.1.1.1 Topic
                        7.1.1.1.1 Información que se recibe [Topic --> info_robot_mover_a_lata]
                            -almacén origen // en teoría lo sé ya
                            -almacén destino                                
                        7.1.1.1.2 Información que se envía [Topic --> info_camara_mover_a_lata]
    

'''


class Navegacion_Robot:
    def __init__(self):
        rospy.init_node("aruco_detector", anonymous=True)

        # Parámetros del sistema
        self.frame_size = (640, 480)

        # Parámetros de calibración de cámara
        self.camera_matrix = 640*480
        self.dist_coeffs = None
        self.calibration_done = False
        
        # Matriz de cámara por defecto (se actualizará durante la calibración)
        self.camera_matrix = np.array([
            [500.0, 0, self.frame_size[0]/2],
            [0, 500.0, self.frame_size[1]/2],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Coeficientes de distorsión por defecto
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # Puntos mundo para realizar la homografía
        self.world_pts = np.array([
            [  600,   600],   # Inferior izquierda
            [  600, 18800],   # Superior izquierda
            [28800,   600],   # Inferior derecha
            [28800, 18800],   # Superior derecha
        ], dtype=np.float32)

        # Diccionario para almacenar información de ArUcos por zona
        self.info_zonas = {
            1: {'arucos': [], 'centros': []},
            2: {'arucos': [], 'centros': []},
            3: {'arucos': [], 'centros': []},
            4: {'arucos': [], 'centros': []},
            5: {'arucos': [], 'centros': []},
            6: {'arucos': [], 'centros': []},
            7: {'arucos': [], 'centros': []},
            8: {'arucos': [], 'centros': []}
        }
        
        self.aruco_ids = []
        self.arucos_medio = {
                            20:[],
                            21:[],
                            22:[],
                            23:[],}
        self.centros_medio_IMG_Points = []

        # Subscripciones y publicaciones ROS
        self.sub = rospy.Subscriber("/csi_cam_0/image_raw", Image, self.cb, queue_size=1)
        
        # Topics para comunicación con el robot
        self.mover_lata_pub = rospy.Publisher("/info_camara_mover_lata", String, queue_size=1)
        self.mover_a_lata_pub = rospy.Publisher("/info_camara_mover_a_lata", String, queue_size=1)
        
        # Suscripciones a los mensajes del robot
        self.mover_lata_sub = rospy.Subscriber("/info_robot_mover_lata", String, 
                                            lambda msg: self.procesamiento_info_topics(msg, "mover_lata"), 
                                            queue_size=1)
        self.mover_a_lata_sub = rospy.Subscriber("/info_robot_mover_a_lata", String, 
                                                lambda msg: self.procesamiento_info_topics(msg, "mover_a_lata"), 
                                                queue_size=1)

        # Estado actual del robot
        self.robot_state = {
            'lata_actual': None,     # ID de la lata que lleva el robot
            'almacen_origen': None,  # Zona de origen 
            'almacen_destino': None, # Zona de destino
            'accion': "Parar",       # Acción actual (Recto, Parar, GirarD, etc)
            'modo': None,            # "mover_lata" o "mover_a_lata"
            'ruta': [],              # Lista de pasos para la ruta calculada
            'paso_actual': 0         # Índice del paso actual en la ruta
        }

        self.robot_id = 15
        
        # Variable para almacenar imágenes durante la calibración
        self.calibration_frames = []
        self.calibration_count = 0
        self.max_calibration_frames = 10
        
        # Intenta cargar calibración previa
        self.load_calibration()

    def preprocess_frame(self, frame):
        try:
            if frame is None:
                return None
            if frame.shape[:2] != (self.frame_size[1], self.frame_size[0]):
                frame = cv2.resize(frame, self.frame_size)
            
            # Aplicar undistort si la cámara está calibrada
            if self.calibration_done:
                frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, self.camera_matrix)
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)
            gray = cv2.equalizeHist(gray)
            return gray
        except Exception as e:
            rospy.logerr("Error in preprocess_frame: %s" % str(e))
            return None
    
    def calibrate_camera(self):
        """
        Calibra la cámara utilizando un patrón de tablero de ajedrez 9x6
        """
        try:
            if len(self.calibration_frames) < 5:
                rospy.logwarn("Se necesitan al menos 5 imágenes para calibración, solo hay %d", 
                    len(self.calibration_frames))
                return False
            
            rospy.loginfo("Iniciando calibración con %d imágenes...", len(self.calibration_frames))
            
            # Dimensiones del tablero de ajedrez (puntos internos)
            chessboard_size = (9, 6)  # Número de esquinas internas (ancho, alto)
            square_size = 0.025  # Tamaño de cada cuadrado en metros
            
            # Preparar puntos objeto 3D
            objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size
            
            # Arrays para almacenar puntos objeto y puntos imagen
            objpoints = []  # Puntos 3D en espacio mundo
            imgpoints = []  # Puntos 2D en plano imagen
            
            for frame in self.calibration_frames:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
                
                if ret:
                    # Refinar esquinas encontradas
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    
                    objpoints.append(objp)
                    imgpoints.append(corners2)
                    
                    # Dibujar y mostrar las esquinas
                    img = frame.copy()
                    cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
                    cv2.imshow('Calibration Pattern', img)
                    cv2.waitKey(500)
            
            if len(objpoints) < 5:
                rospy.logwarn("No se detectaron suficientes patrones en las imágenes")
                return False
                
            # Calibrar cámara
            ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None)
            
            if ret:
                rospy.loginfo("Calibración exitosa!")
                rospy.loginfo("Error RMS: %f", ret)
                rospy.loginfo("Camera Matrix: \n%s", self.camera_matrix)
                rospy.loginfo("Distortion Coeffs: \n%s", self.dist_coeffs)
                
                # Guardar calibración
                self.save_calibration()
                self.calibration_done = True
                cv2.destroyWindow('Calibration Pattern')
                return True
            else:
                rospy.logerr("Falló la calibración")
                return False
                
        except Exception as e:
            rospy.logerr("Error en calibración: %s", str(e))
            return False
            
    def calibrate_with_arucos(self):
        """
        Calibra la cámara utilizando un tablero de ArUcos predefinido
        """
        try:
            if len(self.calibration_frames) < 5:
                rospy.logwarn("Se necesitan al menos 5 imágenes para calibración, solo hay %d", len(self.calibration_frames))
                return False
                
            rospy.loginfo("Iniciando calibración con ArUcos usando %d imágenes...", len(self.calibration_frames))

            # Crear tablero ArUco
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            board = cv2.aruco.CharucoBoard_create(7, 5, 0.04, 0.02, aruco_dict)
            
            # Arrays para almacenar puntos
            all_corners = []
            all_ids = []
            
            for frame in self.calibration_frames:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Detectar ArUcos
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
                
                if ids is not None and len(ids) > 0:
                    # Refinar detecciones
                    ret, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(
                        corners, ids, gray, board)
                    
                    if charucoCorners is not None and charucoIds is not None and len(charucoCorners) > 3:
                        all_corners.append(charucoCorners)
                        all_ids.append(charucoIds)
                        
                        # Dibujar detección
                        cv2.aruco.drawDetectedCornersCharuco(frame, charucoCorners, charucoIds)
                
                cv2.imshow('ArUco Calibration', frame)
                cv2.waitKey(500)
            
            if len(all_corners) < 5:
                rospy.logwarn("No se detectaron suficientes patrones ArUco en las imágenes")
                return False
                
            # Calibrar cámara
            ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                all_corners, all_ids, board, gray.shape[::-1], None, None)
                
            if ret:
                rospy.loginfo("Calibración con ArUco exitosa!")
                rospy.loginfo("Error RMS: %f", ret)
                rospy.loginfo("Camera Matrix: \n%s", self.camera_matrix)
                rospy.loginfo("Distortion Coeffs: \n%s", self.dist_coeffs)
                
                # Guardar calibración
                self.save_calibration()
                self.calibration_done = True
                cv2.destroyWindow('ArUco Calibration')
                return True
            else:
                rospy.logerr("Falló la calibración con ArUco")
                return False
                
        except Exception as e:
            rospy.logerr("Error en calibración con ArUco: %s", str(e))
            return False
    
    def save_calibration(self):
        """Guarda los parámetros de calibración en un archivo"""
        try:
            calibration_data = {
                'camera_matrix': self.camera_matrix.tolist(),
                'dist_coeffs': self.dist_coeffs.tolist(),
                'frame_size': self.frame_size
            }
            
            # Guardar en formato YAML o JSON
            np.savez('camera_calibration.npz', 
                    camera_matrix=self.camera_matrix, 
                    dist_coeffs=self.dist_coeffs,
                    frame_size=np.array(self.frame_size))
            
            rospy.loginfo("Parámetros de calibración guardados en 'camera_calibration.npz'")
            return True
        except Exception as e:
            rospy.logerr("Error al guardar calibración: %s", str(e))
            return False
    
    def load_calibration(self):
        """Carga los parámetros de calibración desde un archivo"""
        try:
            if os.path.exists('camera_calibration.npz'):
                data = np.load('camera_calibration.npz')
                self.camera_matrix = data['camera_matrix']
                self.dist_coeffs = data['dist_coeffs']
                saved_size = data['frame_size']
                
                if tuple(saved_size) == self.frame_size:
                    self.calibration_done = True
                    rospy.loginfo("Parámetros de calibración cargados")
                    return True
                else:
                    rospy.logwarn("Tamaño de frame en calibración (%s) no coincide con actual (%s)",tuple(saved_size), self.frame_size)
            return False
        except Exception as e:
            rospy.logerr("Error al cargar calibración: %s", str(e))
            return False
            
    def collect_calibration_frame(self, frame):
        """Añade un frame a la colección para calibración"""
        if frame is not None and self.calibration_count < self.max_calibration_frames:
            self.calibration_frames.append(frame.copy())
            self.calibration_count += 1
            rospy.loginfo("Capturado frame %d/%d para calibración", self.calibration_count, self.max_calibration_frames)
            
            if self.calibration_count >= self.max_calibration_frames:
                rospy.loginfo("Colección de frames completa, iniciando calibración...")
                self.calibrate_with_arucos()  # Preferimos calibración con ArUcos

    def procesamiento_info_topics(self, msg, topic_type):
        """
        Procesa los mensajes recibidos desde los topics del robot y genera las respuestas apropiadas.
        
        Args:
            msg: Mensaje ROS recibido
            topic_type: Tipo de topic ('mover_lata' o 'mover_a_lata')
        """
        try:
            # Intentar parsear como JSON
            try:
                # Convertir a diccionario de Python
                data = json.loads(msg.data)
                
                # Mapear claves si es necesario
                if 'alm1' in data:
                    data['origen'] = data['alm1']
                if 'alm2' in data:
                    data['destino'] = data['alm2']
                
            except ValueError:
                # Si falla, intentar con el formato original clave:valor
                data = {}
                for item in msg.data.split(','):
                    if ':' in item:
                        key, value = item.split(':')
                        data[key.strip()] = value.strip()
            
            rospy.loginfo("Recibido mensaje de %s: %s", topic_type, data)
            
            # Actualizar el estado del robot según el tipo de mensaje
            self.robot_state['modo'] = topic_type
            
            if topic_type == "mover_lata":
                # Mensaje de mover una lata de un almacén a otro
                if 'lata' in data:
                    self.robot_state['lata_actual'] = int(data['lata'])
                if 'origen' in data:
                    self.robot_state['almacen_origen'] = int(data['origen'])
                if 'destino' in data:
                    self.robot_state['almacen_destino'] = int(data['destino'])
                
                # Calcular ruta para mover la lata
                self.calcular_ruta_mover_lata()
                
            elif topic_type == "mover_a_lata":
                # Mensaje de moverse hacia una lata
                if 'origen' in data:
                    self.robot_state['almacen_origen'] = int(data['origen'])
                if 'destino' in data:
                    self.robot_state['almacen_destino'] = int(data['destino'])
                
                # Calcular ruta para ir a buscar la lata
                self.calcular_ruta_mover_a_lata()
            
            # Ejecutar el siguiente paso de la ruta calculada
            self.ejecutar_siguiente_accion()
            
        except Exception as e:
            rospy.logerr("Error procesando mensaje %s: %s", topic_type, str(e))
    
    
    def camino_a_comandos(self, camino, zonas_posiciones):
        """
        Convierte un camino de zonas en comandos de movimiento para el robot.
        
        Args:
            camino: Lista de zonas [origen, ..., destino]
            zonas_posiciones: Diccionario {zona_id: (x, y)}
            
        Returns:
            List[str]: Lista de comandos (Recto, GirarD, GirarI, etc.)
        """
        if len(camino) <= 1:
            return ["Parar"]
            
        comandos = []
        
        # Orientación inicial (asumimos que mira hacia el siguiente punto)
        for i in range(len(camino) - 1):
            # Obtener posiciones actuales y siguiente
            zona_actual = camino[i]
            zona_siguiente = camino[i + 1]
            pos_actual = zonas_posiciones[zona_actual]
            pos_siguiente = zonas_posiciones[zona_siguiente]
            
            # Si es la primera zona, salir del almacén
            if i == 0:
                comandos.append("Atras")
            
            # Calcular dirección de movimiento
            dx = pos_siguiente[0] - pos_actual[0]
            dy = pos_siguiente[1] - pos_actual[1]
            
            # Ángulo de movimiento (en radianes)
            angulo = np.arctan2(dy, dx)
            
            # Convertir a grados
            angulo_grados = np.degrees(angulo)
            
            # Decidir giro basado en el ángulo
            if i > 0:  # No girar en la primera iteración
                # Calcular cambio en la dirección
                zona_previa = camino[i-1]
                pos_previa = zonas_posiciones[zona_previa]
                dx_prev = pos_actual[0] - pos_previa[0]
                dy_prev = pos_actual[1] - pos_previa[1]
                angulo_prev = np.degrees(np.arctan2(dy_prev, dx_prev))
                
                # Diferencia en ángulos
                diff_angulo = angulo_grados - angulo_prev
                # Normalizar a rango -180 a 180
                while diff_angulo > 180: diff_angulo -= 360
                while diff_angulo < -180: diff_angulo += 360
                
                # Decidir si girar y en qué dirección
                if -30 <= diff_angulo <= 30:
                    # Ángulos similares, seguir recto
                    pass
                elif diff_angulo > 30:
                    # Giro a la derecha
                    comandos.append("GirarD")
                else:  # diff_angulo < -30
                    # Giro a la izquierda
                    comandos.append("GirarI")
            
            # Avanzar hacia la siguiente zona
            comandos.append("Recto")
        
        # Al llegar al destino
        comandos.append("Parar")
        comandos.append("Destino")
        
        return comandos
    '''
    def get_ruta_predeterminada(self, origen, destino):
        """
        Retorna una ruta predeterminada en caso de que A* falle
        """
        return [
            "Atras",   # Salir del almacén de origen
            "GirarD",  # Orientarse hacia el almacén destino
            "Recto",   # Avanzar hacia el destino
            "Recto",   # Seguir avanzando
            "GirarI",  # Orientarse para entrar al almacén
            "Recto",   # Entrar al almacén
            "Parar",   # Detener para dejar la lata
            "Destino"  # Indicar que hemos llegado
        ]
    '''
    def calcular_ruta_mover_lata(self):
        """
        Calcula la dirección para mover una lata desde origen hasta destino.
        Usa una navegación simple basada en distancias.
        """
        try:
            origen_id = self.robot_state['almacen_origen']
            destino_id = self.robot_state['almacen_destino']
            
            # Verificar que tenemos la información necesaria
            if origen_id is None or destino_id is None:
                rospy.logwarn("Origen o destino no definidos para mover lata")
                return
            
            # Obtener posiciones de origen y destino
            pos_origen = self.obtener_posicion_zona(origen_id)
            pos_destino = self.obtener_posicion_zona(destino_id)
            
            if not pos_origen or not pos_destino:
                rospy.logwarn("No se pueden obtener posiciones para origen o destino")
                return
            
            # Determinamos la posición actual del robot (posición del ArUco del robot)
            pos_robot = None
            # Buscar ArUco del robot (asumimos ID 15, ajusta según tu caso)
            robot_id = 15
            for zona_id, zona_info in self.info_zonas.items():
                for aruco in zona_info['arucos']:
                    if aruco['aruco_id'] == robot_id:
                        pos_robot = aruco['centro']
                        break
            
            # Si no encontramos el robot, asumimos que está en el origen
            if not pos_robot:
                pos_robot = pos_origen
            
            # Calcular dirección basada en posiciones
            comando = self.calcular_direccion_robot(pos_robot, pos_origen, pos_destino)
            
            # Actualizar estado del robot con un solo paso
            self.robot_state['ruta'] = [comando]
            self.robot_state['paso_actual'] = 0
            
            rospy.loginfo("Calculada dirección simple para mover lata: %s", comando)
            
        except Exception as e:
            rospy.logerr("Error calculando dirección mover_lata: %s", str(e))
            self.robot_state['ruta'] = ["Parar"]
            self.robot_state['paso_actual'] = 0
            
    def calcular_ruta_mover_a_lata(self):
        """
        Calcula la dirección para que el robot se mueva hacia una lata.
        Usa una navegación simple basada en distancias.
        """
        try:
            origen_id = self.robot_state['almacen_origen']
            destino_id = self.robot_state['almacen_destino']
            
            # Verificar que tenemos la información necesaria
            if origen_id is None or destino_id is None:
                rospy.logwarn("Origen o destino no definidos para mover a lata")
                return
            
            # Obtener posiciones de origen y destino
            pos_origen = self.obtener_posicion_zona(origen_id)
            pos_destino = self.obtener_posicion_zona(destino_id)
            
            if not pos_origen or not pos_destino:
                rospy.logwarn("No se pueden obtener posiciones para origen o destino")
                return
            
            # Determinamos la posición actual del robot (posición del ArUco del robot)
            pos_robot = None
            # Buscar ArUco del robot (asumimos ID 15, ajusta según tu caso)
            robot_id = 15
            for zona_id, zona_info in self.info_zonas.items():
                for aruco in zona_info['arucos']:
                    if aruco['aruco_id'] == robot_id:
                        pos_robot = aruco['centro']
                        break
            
            # Si no encontramos el robot, asumimos que está en el origen
            if not pos_robot:
                pos_robot = pos_origen
            
            # Calcular dirección basada en posiciones
            comando = self.calcular_direccion_robot(pos_robot, pos_origen, pos_destino)
            
            # Actualizar estado del robot con un solo paso
            self.robot_state['ruta'] = [comando]
            self.robot_state['paso_actual'] = 0
            
            rospy.loginfo("Calculada dirección simple para mover a lata: %s", comando)
            
        except Exception as e:
            rospy.logerr("Error calculando dirección mover_a_lata: %s", str(e))
            self.robot_state['ruta'] = ["Parar"]
            self.robot_state['paso_actual'] = 0
    
    def ejecutar_siguiente_accion(self):
        """
        Calcula y ejecuta la siguiente acción de navegación basada en la posición actual.
        """
        try:
            origen_id = self.robot_state['almacen_origen']
            destino_id = self.robot_state['almacen_destino']
            
            # Verificar que tenemos la información necesaria
            if origen_id is None or destino_id is None:
                rospy.logwarn("Origen o destino no definidos para ejecutar acción")
                return
            
            # Obtener posiciones de origen y destino
            pos_origen = self.obtener_posicion_zona(origen_id)
            pos_destino = self.obtener_posicion_zona(destino_id)
            
            if not pos_origen or not pos_destino:
                rospy.logwarn("No se pueden obtener posiciones para origen o destino")
                return
            
            # Determinamos la posición actual del robot (posición del ArUco del robot)
            pos_robot = self.obtener_posicion_robot()
            
            # Si no encontramos el robot, no podemos navegar
            if not pos_robot:
                rospy.logwarn("No se puede localizar al robot para ejecutar acción")
                accion = "Parar"
            else:
                # Calcular dirección basada en posiciones actuales
                accion = self.calcular_direccion_robot(pos_robot, pos_origen, pos_destino)
            
            # Actualizar estado del robot
            self.robot_state['accion'] = accion
            
            # Publicar la acción en el topic correspondiente
            if self.robot_state['modo'] == "mover_lata":
                mensaje = "accion:{}".format(accion)
                self.mover_lata_pub.publish(String(data=mensaje))
                rospy.loginfo("Publicado en mover_lata: %s", mensaje)
            
            elif self.robot_state['modo'] == "mover_a_lata":
                mensaje = "accion:{}".format(accion)
                self.mover_a_lata_pub.publish(String(data=mensaje))
                rospy.loginfo("Publicado en mover_a_lata: %s", mensaje)
            
            # Verificar si hemos llegado al destino
            if accion == "Destino":
                mensaje = "estado:completado"
                if self.robot_state['modo'] == "mover_lata":
                    self.mover_lata_pub.publish(String(data=mensaje))
                else:
                    self.mover_a_lata_pub.publish(String(data=mensaje))
                rospy.loginfo("Navegación completada: %s", self.robot_state['modo'])
            
        except Exception as e:
            rospy.logerr("Error ejecutando acción: %s", str(e))
    
    def obtener_posicion_zona(self, zona_id):
        """
        Obtiene la posición (centro) de una zona específica basándose en los centros almacenados.
        
        Args:
            zona_id: ID de la zona (1-8)
            
        Returns:
            tuple: (x, y) del centro de la zona, o None si no se encuentra
        """
        try:
            if zona_id not in self.info_zonas:
                return None
                
            zona = self.info_zonas[zona_id]
            centros = zona['centros']
            
            if not centros:
                return None
                
            # Calcular el centro como el promedio de todos los centros en la zona
            # Usar float para asegurar división correcta en Python 2.7
            x_avg = sum(float(c[0]) for c in centros) / len(centros)
            y_avg = sum(float(c[1]) for c in centros) / len(centros)
            
            return (int(x_avg), int(y_avg))
            
        except Exception as e:
            rospy.logerr("Error obteniendo posición para zona %d: %s", zona_id, str(e))
            return None

    def detect_arucos(self, frame):
        try:
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

            if ids is not None:
                for i, id_ in enumerate(ids.flatten()):
                    c = corners[i]
                    centro = tuple(np.mean(c.reshape(4,2), axis=0).astype(int))
                    
                    # Determinar a qué zona pertenece cada ArUco
                    if 9 <= id_ <= 14:
                        zona = id_ - 8  # ArUcos 9-14 corresponden a zonas 1-6
                        aruco_info = {
                            'aruco_id': id_,
                            'centro': centro,
                            'corners': c.reshape(4,2).tolist()
                        }
                        
                        # Añadir el ArUco a la lista de la zona correspondiente
                        self.info_zonas[zona]['arucos'].append(aruco_info)
                        # Añadir el centro a la lista de centros de la zona
                        self.info_zonas[zona]['centros'].append(centro)
                        
                    if 20 <= id_ <= 23:
                        aruco = id_
                        # Almacenar tanto las esquinas como el centro calculado
                        self.arucos_medio[aruco].append({
                            'corners': corners[i],
                            'center': centro
                        })
                    
                    # ArUcos adicionales pueden asignarse a otras zonas
                    # Zona 7 y 8 pueden tener sus propios ArUcos

        except Exception as e:
            rospy.logerr("Error in detect_arucos: %s" % str(e))
            return None

    def calculo_homografia(self, frame):
        try:
            # Verificar que tenemos datos de los 4 ArUcos de referencia
            available_arucos = 0
            for aruco_id in range(20, 24):  # ArUcos 20, 21, 22, 23
                if self.arucos_medio[aruco_id] and len(self.arucos_medio[aruco_id]) > 0:
                    available_arucos += 1
            
            if available_arucos < 4:
                rospy.logwarn("Se necesitan los 4 ArUcos de referencia (20-23). Solo hay %d", available_arucos)
                return None
                
            # Extraer los centros de los 4 ArUcos de referencia
            pts = np.zeros((4, 2), dtype=np.float32)
            for i, aruco_id in enumerate(range(20, 24)):
                # Usar el ArUco más reciente detectado
                aruco_data = self.arucos_medio[aruco_id][-1]
                center = aruco_data['center']
                pts[i] = [center[0], center[1]]
                
                # Dibujar punto de referencia para debug
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(frame, f"Ref {aruco_id}", (center[0], center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Calcular la homografía
            H, status = cv2.findHomography(pts, self.world_pts, cv2.RANSAC, 5.0)
            
            if status.sum() >= 3:  # Al menos 3 buenos puntos
                return H
            else:
                rospy.logwarn("No se pudo calcular una buena homografía")
                return None
                
        except Exception as e:
            rospy.logerr("Error en cálculo de homografía: %s", str(e))
            return None

    def creacion_centros(self, frame, zona):
        try:
            if zona in self.info_zonas:
                # Dibujar centros de ArUcos específicos de la zona
                for centro in self.info_zonas[zona]['centros']:
                    # Dibujar centro en la imagen
                    cv2.circle(frame, centro, 4, (0, 255, 0), -1)
                
                # Si necesitamos etiquetar con ID, usamos la información de arucos
                for aruco in self.info_zonas[zona]['arucos']:
                    # Añadir etiqueta con ID y zona
                    center = aruco['centro']
                    label = f"ID:{aruco['aruco_id']} Z:{zona}"
                    cv2.putText(frame, label, (center[0] + 10, center[1]), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Usar directamente los centros almacenados en arucos_medio
            self.centros_medio_IMG_Points = []
            for aruco_id, aruco_data_list in self.arucos_medio.items():
                for aruco_data in aruco_data_list:
                    center = aruco_data['center']
                    self.centros_medio_IMG_Points.append(center)
                    cv2.circle(frame, center, 4, (255, 0, 0), -1)
                    
                    # Añadir etiqueta con el ID del aruco
                    cv2.putText(frame, f"ID:{aruco_id}", (center[0] + 10, center[1]), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            return frame
        except Exception as e:
            rospy.logerr("Error in creacion_centros: %s" % str(e))
            return frame

    def calculate_center(self, corners):
        try:
            if isinstance(corners, list) and len(corners) > 0:
                if isinstance(corners[0], np.ndarray):
                    # Si es formato de detectMarkers
                    marker_corners = corners[0]
                    center_x = int(np.mean(marker_corners[:, 0]))
                    center_y = int(np.mean(marker_corners[:, 1]))
                    return (center_x, center_y)
                else:
                    # Si son puntos directos
                    corners_array = np.array(corners)
                    center_x = int(np.mean(corners_array[:, 0]))
                    center_y = int(np.mean(corners_array[:, 1]))
                    return (center_x, center_y)
            return (0, 0)  # Valor por defecto si hay error
        except Exception as e:
            rospy.logerr("Error in calculate_center: %s" % str(e))
            return (0, 0)

    def calculo_distancia_robot_lata(self):
        try:
            # Esta función necesita ser actualizada según tus nuevas estructuras de datos
            # Ejemplo simplificado:
            if hasattr(self, 'centros_robot') and hasattr(self, 'centros_lata') and \
                self.centros_robot and self.centros_lata:
                robot = np.array(self.centros_robot[-1])
                lata = np.array(self.centros_lata[-1])
                distancia = np.linalg.norm(robot - lata)
                return distancia
            return 1e9
        except Exception as e:
            rospy.logerr("Error in calculo_distancia: %s" % str(e))
            return 1e9
            
    def cb(self, msg):
        try:
            # Convertir mensaje ROS a imagen OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Si estamos en proceso de calibración, recoger frames
            if not self.calibration_done and self.calibration_count < self.max_calibration_frames:
                # Cada 30 frames capturamos uno para calibración
                if self.calibration_count % 30 == 0:
                    self.collect_calibration_frame(frame)
            
            # Limpiar datos de detecciones previas
            for zona_id in self.info_zonas:
                self.info_zonas[zona_id]['arucos'] = []
                self.info_zonas[zona_id]['centros'] = []
            
            for aruco_id in self.arucos_medio:
                self.arucos_medio[aruco_id] = []
                    
            # Ejecutar el pipeline de procesamiento
            preprocessed = self.preprocess_frame(frame)
            if preprocessed is not None:
                self.detect_arucos(preprocessed)
                
                # Aplicar homografía si la tenemos
                warped_frame = None
                H = self.calculo_homografia(frame)
                if H is not None:
                    warped_frame = cv2.warpPerspective(frame, H, (1000, 1000))
                    
                    # Procesar cada zona en la vista transformada
                    for zona in range(1, 9):
                        warped_frame = self.creacion_centros(warped_frame, zona)
                    
                    # Si estamos en modo de navegación, ejecutar siguiente acción
                    if self.robot_state['modo'] in ["mover_lata", "mover_a_lata"]:
                        self.ejecutar_siguiente_accion()
                    
                    # Visualizar el vector de dirección si estamos en modo navegación
                    pos_robot = self.obtener_posicion_robot()
                    if pos_robot and self.robot_state['modo'] in ["mover_lata", "mover_a_lata"]:
                        destino_id = self.robot_state['almacen_destino']
                        pos_destino = self.obtener_posicion_zona(destino_id)
                        
                        if pos_destino:
                            # Transformar posiciones al espacio warped
                            robot_pts = np.array([[pos_robot]], dtype=np.float32)
                            destino_pts = np.array([[pos_destino]], dtype=np.float32)
                            
                            warped_robot = cv2.perspectiveTransform(robot_pts, H)[0][0]
                            warped_destino = cv2.perspectiveTransform(destino_pts, H)[0][0]
                            
                            # Convertir a tuplas de enteros
                            warped_robot = (int(warped_robot[0]), int(warped_robot[1]))
                            warped_destino = (int(warped_destino[0]), int(warped_destino[1]))
                            
                            # Dibujar flechas de dirección
                            cv2.arrowedLine(frame, pos_robot, pos_destino, (0, 0, 255), 2)
                            cv2.arrowedLine(warped_frame, warped_robot, warped_destino, (0, 0, 255), 2)
                    
                    # Mostrar warped frame
                    cv2.imshow("Vista Cenital", warped_frame)
                
                # También procesar el frame original para visualización
                frame_visualizacion = frame.copy()
                for zona in range(1, 9):
                    frame_visualizacion = self.creacion_centros(frame_visualizacion, zona)
                
                # Mostrar resultado
                cv2.imshow("ArUco Detection", frame_visualizacion)
                cv2.waitKey(1)
                
        except Exception as e:
            rospy.logerr("Error in callback: %s" % str(e))

    def calcular_distancia_entre_puntos(self, punto1, punto2):
        """
        Calcula la distancia euclidiana entre dos puntos.
        
        Args:
            punto1: Tupla (x, y) del primer punto
            punto2: Tupla (x, y) del segundo punto
            
        Returns:
            float: Distancia entre los puntos
        """
        return np.sqrt(float(punto2[0] - punto1[0])**2 + float(punto2[1] - punto1[1])**2)
        
    def calcular_direccion_robot(self, pos_actual, pos_origen, pos_destino):
        """
        Calcula la dirección que debe tomar el robot basándose en su posición actual,
        el origen y el destino.
        
        Args:
            pos_actual: Tupla (x, y) de la posición actual del robot
            pos_origen: Tupla (x, y) de la posición de origen
            pos_destino: Tupla (x, y) de la posición de destino
            
        Returns:
            str: Comando de dirección ("Recto", "GirarD", "GirarI", "Parar", "Destino")
        """
        # Calcular distancias
        distancia_al_origen = self.calcular_distancia_entre_puntos(pos_actual, pos_origen)
        distancia_al_destino = self.calcular_distancia_entre_puntos(pos_actual, pos_destino)
        
        # Calcular dirección actual del robot (vector desde origen hacia posición actual)
        vector_actual = (pos_actual[0] - pos_origen[0], pos_actual[1] - pos_origen[1])
        
        # Calcular dirección ideal (vector desde posición actual hacia destino)
        vector_ideal = (pos_destino[0] - pos_actual[0], pos_destino[1] - pos_actual[1])
        
        # Calcular ángulo entre los vectores (producto punto)
        magnitud_actual = np.sqrt(float(vector_actual[0])**2 + float(vector_actual[1])**2)
        magnitud_ideal = np.sqrt(float(vector_ideal[0])**2 + float(vector_ideal[1])**2)
        
        # Evitar división por cero
        if magnitud_actual == 0 or magnitud_ideal == 0:
            return "Parar"
            
        producto_punto = vector_actual[0]*vector_ideal[0] + vector_actual[1]*vector_ideal[1]
        coseno_angulo = float(producto_punto) / (magnitud_actual * magnitud_ideal)
        
        # Ajustar para evitar errores numéricos
        coseno_angulo = max(-1, min(1, coseno_angulo))
        angulo_rad = np.arccos(coseno_angulo)
        angulo_grados = np.degrees(angulo_rad)
        
        # Determinar si girar a la derecha o izquierda (producto cruz)
        producto_cruz = vector_actual[0]*vector_ideal[1] - vector_actual[1]*vector_ideal[0]
        
        # Comprobar si estamos cerca del destino
        umbral_destino = 50  # ajustar según sea necesario
        if distancia_al_destino < umbral_destino:
            return "Destino"
        
        # Decidir acción basada en ángulo
        if angulo_grados < 20:
            return "Recto"  # Ángulo pequeño, seguir recto
        elif producto_cruz > 0:
            return "GirarD"  # Girar a la derecha
        else:
            return "GirarI"  # Girar a la izquierda

    def obtener_posicion_robot(self):
        """
        Obtiene la posición actual del robot basada en la detección del ArUco del robot.
        
        Returns:
            tuple: (x, y) del centro del robot, o None si no se encuentra
        """
        try:
            for zona_id, zona_info in self.info_zonas.items():
                for aruco in zona_info['arucos']:
                    if aruco['aruco_id'] == self.robot_id:
                        return aruco['centro']
            return None
        except Exception as e:
            rospy.logerr("Error obteniendo posición del robot: %s", str(e))
            return None
