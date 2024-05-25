import serial
import time
import json
from datetime import datetime


ser = serial.Serial('COM9', 9600) 
time.sleep(2)

setpoint = 30.0
kp = 0.5
ki = 0.05
kd = 0.1

integral = 0
last_error = 0
last_time = time.time()

data_log = []

def pid_control(error, time_change):
    global integral, last_error
    integral += error * time_change
    integral = max(min(integral, 50), -50)  # Limita el término integral entre -50 y 50
    derivative = (error - last_error) / time_change
    output = kp * error + ki * integral + kd * derivative
    last_error = error
    return output

while True:
    try:
        # Leer la línea desde Arduino
        line = ser.readline().decode('utf-8').strip()
        if "Distance:" in line:
            distance = float(line.split(":")[1].strip())
            error = setpoint - distance
            current_time = time.time()
            time_change = current_time - last_time
            if time_change >= 0.05:  # Actualización a 50 ms
                output = pid_control(error, time_change)
                # Envía tanto el error como el valor del PID a Arduino
                ser.write(f"{error},{output}\n".encode('utf-8'))
                last_time = current_time

            # Obtener la fecha y hora actual
            now = datetime.now()
            timestamp = now.strftime("%Y-%m-%d %H:%M:%S")

            # Capturar la fecha y hora junto con la distancia
            data_entry = {
                "timestamp": timestamp,
                "distance": distance
            }
            # Agregar entrada de datos al registro
            data_log.append(data_entry)
            print(f"Distance: {distance} cm, Timestamp: {timestamp}")

            # Guardar datos en archivo JSON periódicamente
            if len(data_log) % 10 == 0:  # Guarda cada 10 lecturas
                with open('sensor_data.json', 'w') as f:
                    json.dump(data_log, f, indent=4)

    except Exception as e:
        print(f"Error: {e}")
