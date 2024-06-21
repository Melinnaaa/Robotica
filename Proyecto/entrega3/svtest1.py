import cv2
import logging
import threading
import time
from flask import Flask, Response, request, jsonify
from flask_cors import CORS
import serial
import json
import numpy as np
from queue import PriorityQueue
import subprocess
import random

app = Flask(__name__)

# Configurar CORS para permitir orígenes específicos
cors = CORS(app, resources={
    r"/*": {
        "origins": [
            "http://localhost:8100",
            "http://186.78.106.238:8100"
        ],
        "supports_credentials": True
    }
})

logging.basicConfig(level=logging.DEBUG)

# Configurar el puerto serial para la comunicación con el Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# Variables globales
sensor_data = {}  # Datos de los sensores recibidos del Arduino
latest_frame = None  # Último frame capturado del video
frame_lock = threading.Lock()  # Bloqueo para manejar acceso concurrente al frame
streaming = False  # Estado de la transmisión de video
detecting = False  # Estado de la detección de mascotas
autonomous_mode = False  # Modo autónomo activado
astar_mode = False  # Modo A* activado
detection_thread = None  # Hilo para la detección autónoma
libcamera_process = None  # Proceso de libcamera
opencv_capture = None  # Captura de video con OpenCV
last_pet_detection_time = 0  # Última vez que se detectó una mascota
arbitrary_movement_active = False  # Movimiento arbitrario activado
stop_arbitrary_movement_event = threading.Event()  # Evento para detener movimiento arbitrario

# Mapa 6x6 sin bordes
# 0 = espacio libre
# 1 = obstáculo
map_grid = np.array([
    [0, 0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 1, 0],
    [0, 0, 0, 0, 1, 0]
])

robot_position = (0, 1)  # Posición inicial del robot
goal_position = (5, 3)   # Posición objetivo del robot

# Función para leer datos del puerto serial
def read_from_serial():
    global sensor_data
    while True:
        try:
            line = ser.readline()  # Leer una línea del puerto serial
            if line:
                try:
                    decoded_line = line.decode('utf-8').rstrip()  # Decodificar la línea recibida
                except UnicodeDecodeError:
                    try:
                        decoded_line = line.decode('latin-1').rstrip()
                    except UnicodeDecodeError:
                        decoded_line = line.decode('ascii', errors='ignore').rstrip()

                # Convertir los datos del sensor a formato JSON
                sensor_data = json.loads(decoded_line)
                print(f'Datos del sensor recibidos: {sensor_data}')
        except json.JSONDecodeError as e:
            print(f'Error al analizar los datos del sensor: {e}')
        except serial.SerialException as e:
            print(f'Error al leer desde el puerto serial: {e}')

# Iniciar el hilo para leer desde el puerto serial
serial_thread = threading.Thread(target=read_from_serial)
serial_thread.daemon = True
serial_thread.start()

# Ruta POST para recibir datos de sensores
@app.route('/api/sensors', methods=['POST'])
def receive_sensor_data():
    global sensor_data
    sensor_data = request.json  # Obtener los datos JSON del cuerpo de la solicitud
    print(f'Datos del sensor recibidos: {sensor_data}')
    return jsonify(status='success')

# Ruta GET para obtener datos de sensores
@app.route('/api/sensors', methods=['GET'])
def get_sensor_data():
    return jsonify(sensor_data)

# Ruta POST para enviar comandos de control al robot
@app.route('/api/control', methods=['POST'])
def control_robot():
    command = request.json.get('command')  # Obtener el comando del cuerpo de la solicitud
    print(f'Comando de control recibido: {command}')
    try:
        ser.write((command + '\n').encode())  # Enviar el comando al puerto serial
        return jsonify(status='success')
    except serial.SerialException as e:
        return jsonify(status='error', message=str(e)), 500

# Ruta POST para enviar un comando de keep-alive al robot
@app.route('/api/keep-alive', methods=['POST'])
def keep_alive():
    try:
        ser.write("KEEP_ALIVE\n".encode())  # Enviar comando de keep-alive al puerto serial
        return jsonify(status='success')
    except serial.SerialException as e:
        return jsonify(status='error', message=str(e)), 500

# Función para iniciar libcamera
def start_libcamera():
    global libcamera_process
    if not libcamera_process:
        libcamera_process = subprocess.Popen(['libcamera-vid', '-t', '0', '--inline', '--listen', '-o', 'tcp://0.0.0.0:5001'])
        time.sleep(5)

# Función para detener libcamera
def stop_libcamera():
    global libcamera_process
    if libcamera_process:
        libcamera_process.terminate()
        libcamera_process = None

# Función para adquirir la captura de video usando OpenCV
def acquire_opencv_capture():
    global opencv_capture
    if not opencv_capture:
        opencv_capture = cv2.VideoCapture("tcp://0.0.0.0:5001")
        if not opencv_capture.isOpened():
            app.logger.error("No se pudo abrir el stream de video en 'tcp://0.0.0.0:5001'")
            opencv_capture = None

# Función para mantener la captura de video activa
def keep_opencv_capture_alive():
    global opencv_capture
    while True:
        if opencv_capture is None or not opencv_capture.isOpened():
            acquire_opencv_capture()
        time.sleep(1)

# Iniciar hilo para mantener la captura de video activa
capture_thread = threading.Thread(target=keep_opencv_capture_alive)
capture_thread.daemon = True
capture_thread.start()

# Función para generar frames para el stream de video
def generate_frames():
    global latest_frame, opencv_capture

    while streaming:  # Continuar solo si el streaming está activo
        if not opencv_capture:
            acquire_opencv_capture()

        success, frame = opencv_capture.read()
        if not success:
            app.logger.error("Error al capturar imagen del stream")
            break
        else:
            with frame_lock:
                latest_frame = cv2.rotate(frame, cv2.ROTATE_180)  # Rotar la imagen 180 grados

            ret, buffer = cv2.imencode('.jpg', latest_frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Ruta para el feed de video
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Ruta para iniciar el streaming
@app.route('/api/start_stream')
def start_stream():
    global streaming, detecting, astar_mode
    if detecting:
        stop_detection()  # Detener la detección si está activa

    streaming = True
    astar_mode = False
    print("Streaming iniciado")
    return jsonify(status='streaming started')

# Ruta para detener el streaming
@app.route('/api/stop_stream')
def stop_stream():
    global streaming
    streaming = False
    print("Streaming detenido")
    return jsonify(status='streaming stopped')

# Ruta para iniciar la detección
@app.route('/api/start_detection')
def start_detection():
    global detecting, streaming, autonomous_mode, astar_mode, detection_thread, last_pet_detection_time, arbitrary_movement_active, stop_arbitrary_movement_event
    if streaming:
        stop_stream()  # Detener el streaming si está activo

    detecting = True
    autonomous_mode = True
    astar_mode = False
    arbitrary_movement_active = False
    stop_arbitrary_movement_event.clear()
    last_pet_detection_time = time.time()  # Inicializar la variable cuando comience la detección
    start_libcamera()
    print("Detección iniciada")
    if not detection_thread or not detection_thread.is_alive():
        detection_thread = threading.Thread(target=autonomous_navigation)
        detection_thread.daemon = True
        detection_thread.start()
    return jsonify(status='detection started')

# Ruta para detener la detección
@app.route('/api/stop_detection')
def stop_detection():
    global detecting, autonomous_mode, arbitrary_movement_active, stop_arbitrary_movement_event
    detecting = False
    autonomous_mode = False
    arbitrary_movement_active = False
    stop_arbitrary_movement_event.set()
    print("Detección detenida")
    return jsonify(status='detection stopped')

# Ruta para iniciar el modo A*
@app.route('/api/start_astar')
def start_astar():
    global astar_mode, autonomous_mode, detecting, streaming, detection_thread, arbitrary_movement_active, stop_arbitrary_movement_event, robot_position
    if streaming:
        stop_stream()  # Detener el streaming si está activo
    if detecting:
        stop_detection()  # Detener la detección si está activa

    autonomous_mode = False
    astar_mode = True
    detecting = False
    streaming = False
    arbitrary_movement_active = False
    stop_arbitrary_movement_event.set()
    robot_position = (0, 1)  # Reiniciar la posición del robot
    start_libcamera()
    print("A* iniciado")
    if not detection_thread or not detection_thread.is_alive():
        detection_thread = threading.Thread(target=autonomous_navigation)
        detection_thread.daemon = True
        detection_thread.start()
    return jsonify(status='A* started')

# Ruta para detener el modo A*
@app.route('/api/stop_astar')
def stop_astar():
    global astar_mode
    astar_mode = False
    print("A* detenido")
    return jsonify(status='A* stopped')

# Cargar los modelos MobileNet SSD
prototxt_path = "deploy.prototxt"  # Ruta al archivo .prototxt
model_path = "mobilenet_iter_73000.caffemodel"  # Ruta al archivo .caffemodel
net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)

# Definir las clases que MobileNet SSD puede detectar
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

# Clases que queremos detectar
TARGET_CLASSES = ["cat", "dog", "person"]

# Función para detectar las clases objetivo en el frame
def detect_target_classes(frame):
    global tracker, tracker_initialized, last_pet_detection_time
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    max_confidence = 0
    best_box = None

    detected_classes = []

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.2:
            idx = int(detections[0, 0, i, 1])
            if CLASSES[idx] in TARGET_CLASSES:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                if confidence > max_confidence:
                    max_confidence = confidence
                    best_box = (startX, startY, endX - startX, endY - startY)
                detected_classes.append(CLASSES[idx])

    if best_box is not None:
        tracker = cv2.TrackerCSRT_create()
        tracker.init(frame, best_box)
        tracker_initialized = True
        last_pet_detection_time = time.time()  # Actualizar el tiempo de detección de la mascota

    return detected_classes, best_box

# Función heurística para A* (distancia Manhattan)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Algoritmo A* para encontrar el camino más corto con movimientos diagonales
def a_star_search(start, goal, grid):
    # Lista de posibles movimientos incluyendo movimientos diagonales
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
    close_set = set()  # Conjunto de nodos evaluados
    came_from = {}  # Diccionario para reconstruir el camino
    gscore = {start: 0}  # Costo desde el inicio hasta el nodo actual
    fscore = {start: heuristic(start, goal)}  # Costo estimado total desde el inicio hasta el objetivo
    oheap = PriorityQueue()  # Cola de prioridad para seleccionar el nodo con el menor costo estimado
    oheap.put((fscore[start], start))
    
    while not oheap.empty():
        current = oheap.get()[1]  # Seleccionar el nodo con el menor costo estimado

        # Si el nodo actual es el objetivo, reconstruir y devolver el camino
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)  # Añadir el punto de inicio al camino
            data.reverse()  # Invertir el camino
            return data

        close_set.add(current)  # Añadir el nodo actual al conjunto de nodos evaluados
        for i, j in neighbors:  # Evaluar cada vecino del nodo actual
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            # Comprobar si el vecino está dentro de los límites del grid y no es un obstáculo
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
            
            # Si el vecino ya ha sido evaluado y el nuevo costo no es mejor, continuar
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            
            # Si el nuevo camino al vecino es mejor, actualizar los costos y añadir a la cola de prioridad
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap.queue]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                oheap.put((fscore[neighbor], neighbor))
    
    return []  # Retornar lista vacía si no se encuentra un camino

# Función para mover el robot al siguiente paso en el camino calculado por A*
def move_robot_to_goal(robot_position, goal_position, map_grid):
    path = a_star_search(robot_position, goal_position, map_grid)
    if path:
        if len(path) > 1:
            next_step = path[1]  # La primera posición es la actual, la segunda es el siguiente paso
            x, y = robot_position
            nx, ny = next_step
            # Determinar la dirección del movimiento y enviar el comando correspondiente
            if nx > x and ny == y:
                ser.write("FORWARD\n".encode())
            elif nx < x and ny == y:
                ser.write("BACKWARD\n".encode())
            elif ny > y and nx == x:
                ser.write("RIGHT\n".encode())
            elif ny < y and nx == x:
                ser.write("LEFT\n".encode())
            elif nx > x and ny > y:
                ser.write("FORWARD_RIGHT\n".encode())  
            elif nx > x and ny < y:
                ser.write("FORWARD_LEFT\n".encode())   
            elif nx < x and ny > y:
                ser.write("BACKWARD_RIGHT\n".encode()) 
            elif nx < x and ny < y:
                ser.write("BACKWARD_LEFT\n".encode()) 
            return next_step
        else:
            print("El camino encontrado tiene menos pasos de los esperados.")
    else:
        print("No se encontró un camino. Realizando un movimiento aleatorio.")
        random_movement = random.choice(["LEFT", "RIGHT", "FORWARD"])
        ser.write(f"{random_movement}\n".encode())
    return robot_position

# Función para el movimiento arbitrario del robot cuando no se detecta la mascota
def arbitrary_movement():
    global obstacle_detected, autonomous_mode, arbitrary_movement_active, stop_arbitrary_movement_event
    obstacle_detected = False
    movements = ["FORWARD", "BACKWARD"]
    arbitrary_movement_active = True
    while autonomous_mode and not stop_arbitrary_movement_event.is_set():
        movement = random.choice(movements)
        print(f"Moviéndose {movement.lower()} por 4 segundos")
        ser.write(f"{movement}\n".encode())
        start_time = time.time()
        while time.time() - start_time < 4:
            time.sleep(0.1)
            if sensor_data['irValue'] == 0 or sensor_data['distance'] < 10:
                print("Obstáculo detectado, cambiando dirección")
                obstacle_detected = True
                break

            # Verificar detección de mascota durante el movimiento arbitrario
            with frame_lock:
                frame = latest_frame.copy()
            detected_classes, best_box = detect_target_classes(frame)
            if detected_classes:
                print("Mascota detectada durante movimiento arbitrario")
                stop_arbitrary_movement_event.set()
                break

        if obstacle_detected:
            direction = random.choice(["LEFT", "RIGHT"])
            print(f"Girando a la {direction.lower()} hasta que no haya obstáculos")
            ser.write(f"{direction}\n".encode())
            while sensor_data['irValue'] == 0 or sensor_data['distance'] < 10:
                time.sleep(0.1)
            obstacle_detected = False

# Función para la navegación autónoma del robot
def autonomous_navigation():
    global latest_frame, autonomous_mode, last_pet_detection_time, robot_position, astar_mode, arbitrary_movement_active, stop_arbitrary_movement_event
    
    while autonomous_mode:
        if latest_frame is not None:
            with frame_lock:
                frame = latest_frame.copy()

            print("Comenzando detección de animales")
            detected_classes, best_box = detect_target_classes(frame)
            current_time = time.time()

            if detected_classes:
                for detected_class in detected_classes:
                    print(f"Detectado {detected_class}")
                
                if best_box:
                    startX, startY, width, height = best_box
                    center_x = startX + width // 2

                    frame_center_x = frame.shape[1] // 2

                    if center_x < frame_center_x - 50:
                        ser.write("pet:left\n".encode())
                        print("Mascota detectada a la izquierda")
                    elif center_x > frame_center_x + 50:
                        ser.write("pet:right\n".encode())
                        print("Mascota detectada a la derecha")
                    else:
                        ser.write("pet:forward\n".encode())
                        print("Mascota detectada adelante")
                    last_pet_detection_time = time.time()  # Actualizar el tiempo de detección de la mascota

            elif current_time - last_pet_detection_time > 5:  # Si no se detecta la mascota por 5 segundos
                print("Mascota no detectada por 5 segundos, iniciando movimiento autónomo.")
                stop_arbitrary_movement_event.clear()
                arbitrary_movement()
            else:
                print("Continuando detección de animales")

            time.sleep(0.1)  # Pequeña demora para evitar sobrecarga de CPU

    while astar_mode:
        if robot_position == goal_position:
            print("Meta alcanzada. Deteniendo robot y desactivando A*.")
            ser.write("STOP\n".encode())  # Enviar comando para detener el robot
            astar_mode = False  # Desactivar modo A*
            break
        
        robot_position = move_robot_to_goal(robot_position, goal_position, map_grid)
        print(f"Nueva posición del robot: {robot_position}")
        time.sleep(1)

if __name__ == "__main__":
    start_libcamera() # Se inicia libcamera al inicio del servidor
    app.run(host='0.0.0.0', port=3001)
